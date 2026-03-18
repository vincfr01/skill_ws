import os
from typing import Set, Tuple, Iterable, Optional

import rclpy
from rclpy.node import Node

from owlready2 import get_ontology
from ament_index_python.packages import get_package_share_directory

from robot_interfaces.srv import CanExecuteTask


class OntologyReasoner(Node):
    def __init__(self):
        super().__init__("ontology_reasoner")


        share_dir = get_package_share_directory("robot_semantics")
        ontology_path = os.path.join(share_dir, "ontology", "robot.rdf")

        if not os.path.exists(ontology_path):
            self.get_logger().error(f"Ontology file not found: {ontology_path}")
            raise FileNotFoundError(ontology_path)

        self.get_logger().info(f"Loading ontology: {ontology_path}")
        self.onto = get_ontology(f"file://{ontology_path}").load()


        try:
            from owlready2 import sync_reasoner_pellet

            self.get_logger().info("Running reasoner (Pellet)...")
            sync_reasoner_pellet(
                [self.onto],
                infer_property_values=True,
                infer_data_property_values=True,
            )
            self.get_logger().info("Reasoner finished.")
        except Exception as e:
            self.get_logger().warn(
                "Pellet reasoner not available, continuing without reasoning. "
                f"Details: {e}"
            )

        self.srv = self.create_service(
            CanExecuteTask, "can_execute_task", self.can_execute_cb
        )
        self.get_logger().info("Service '/can_execute_task' ready.")


    def _class_axioms(self, cls) -> Iterable:
        """Return asserted axioms we want to inspect: is_a + equivalent_to."""
        axioms = []
        if hasattr(cls, "is_a"):
            axioms.extend(list(cls.is_a))
        if hasattr(cls, "equivalent_to"):
            axioms.extend(list(cls.equivalent_to))
        return axioms

    def _collect_some_fillers(self, expr, prop_name: str, out: Set) -> None:
        """
        Recursively collect fillers X from expressions of the form:
          <prop_name> some X

        Also descends into AND / intersection expressions that may contain such
        restrictions (common when Protégé builds one combined axiom).
        """

        if hasattr(expr, "property") and getattr(expr, "property") is not None:
            if expr.property.name == prop_name:
                val = getattr(expr, "value", None)
                if val is not None:
                    out.add(val)
            return

        classes = getattr(expr, "Classes", None) or getattr(expr, "classes", None)
        if classes:
            for subexpr in classes:
                self._collect_some_fillers(subexpr, prop_name, out)

    def _some_restrictions(self, cls, prop_name: str) -> Set:
        """Collect all fillers from '<prop_name> some X' restrictions on cls."""
        fillers = set()
        for ax in self._class_axioms(cls):
            self._collect_some_fillers(ax, prop_name, fillers)
        return fillers

    def _collect_required_skills(self, skill_cls) -> Set:
        """
        Collect all skills required transitively by a skill class via:
          requiresSkill some <Skill>

        Includes the start skill itself in the returned set.
        """
        visited = set()
        stack = [skill_cls]

        while stack:
            current = stack.pop()
            if current in visited:
                continue
            visited.add(current)

            req = self._some_restrictions(current, "requiresSkill")
            for s in req:
                if s not in visited:
                    stack.append(s)

        return visited

    def _collect_required_capabilities_for_skill(self, skill_cls) -> Tuple[Set, Set]:
        """
        For a given skill class, resolve all required (sub)skills, then collect
        required capabilities across all of them via:
          requiresCapability some <CapabilityClass>
        """
        all_skills = self._collect_required_skills(skill_cls)

        required_caps = set()
        for s in all_skills:
            required_caps |= self._some_restrictions(s, "requiresCapability")

        return required_caps, all_skills

    def _collect_robot_capability_classes(self, robot_individual) -> Set:
        """
        Collect the robot's capability classes based on:
          robot_individual.hasCapability -> list of individuals
          each capability individual has types in .is_a (classes)
        """
        robot_caps = set()
        if hasattr(robot_individual, "hasCapability"):
            for cap_ind in robot_individual.hasCapability:
                if hasattr(cap_ind, "is_a"):
                    for c in cap_ind.is_a:
                        robot_caps.add(c)
        return robot_caps

    def can_execute_cb(self, request: CanExecuteTask.Request, response: CanExecuteTask.Response):
        robot_id = request.robot_id
        skill_type = request.task_type

        robot = getattr(self.onto, robot_id, None)
        if robot is None:
            response.can_execute = False
            response.reason = f"Robot '{robot_id}' not found in ontology."
            return response

        skill_cls = getattr(self.onto, skill_type, None)
        if skill_cls is None:
            response.can_execute = False
            response.reason = f"Skill class '{skill_type}' not found in ontology."
            return response

        required_caps, used_skills = self._collect_required_capabilities_for_skill(skill_cls)

        if not required_caps:
            response.can_execute = False
            response.reason = (
                f"Skill '{skill_type}' has no requiresCapability restrictions "
                f"(directly or via requiresSkill)."
            )
            return response

        robot_caps = self._collect_robot_capability_classes(robot)

        missing = sorted([req.name for req in required_caps if req not in robot_caps])

        self.get_logger().info(
            f"[Query] robot={robot_id} skill={skill_type} "
            f"used_skills=[{', '.join(sorted([s.name for s in used_skills]))}] "
            f"required_caps=[{', '.join(sorted([c.name for c in required_caps]))}] "
            f"robot_caps=[{', '.join(sorted([c.name for c in robot_caps]))}]"
        )

        if missing:
            response.can_execute = False
            response.reason = f"Missing capabilities: {', '.join(missing)}"
        else:
            response.can_execute = True
            response.reason = "OK"

        return response


def main():
    rclpy.init()
    node = OntologyReasoner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()