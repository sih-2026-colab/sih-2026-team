"""
Sample Scene Utility Module

This module provides utilities for creating and managing sample scenes
for the autonomous driving simulation and testing.
"""


class SampleScene:
    """
    A class to represent and manage sample scenes for autonomous driving.
    """

    def __init__(self, scene_name: str, description: str = ""):
        """
        Initialize a sample scene.

        Args:
            scene_name (str): The name of the scene
            description (str): Optional description of the scene
        """
        self.scene_name = scene_name
        self.description = description
        self.entities = []
        self.obstacles = []
        self.traffic_rules = []

    def add_entity(self, entity):
        """Add an entity to the scene."""
        self.entities.append(entity)

    def add_obstacle(self, obstacle):
        """Add an obstacle to the scene."""
        self.obstacles.append(obstacle)

    def add_traffic_rule(self, rule):
        """Add a traffic rule to the scene."""
        self.traffic_rules.append(rule)

    def get_scene_info(self):
        """Get information about the scene."""
        return {
            "name": self.scene_name,
            "description": self.description,
            "num_entities": len(self.entities),
            "num_obstacles": len(self.obstacles),
            "num_traffic_rules": len(self.traffic_rules),
        }


def create_sample_scene(scene_type: str):
    """
    Create a sample scene based on the specified type.

    Args:
        scene_type (str): Type of scene to create

    Returns:
        SampleScene: A configured sample scene
    """
    scenes = {
        "urban_intersection": SampleScene(
            "Urban Intersection",
            "A busy urban intersection with traffic lights and multiple vehicles"
        ),
        "highway": SampleScene(
            "Highway",
            "A multi-lane highway with high-speed traffic"
        ),
        "residential": SampleScene(
            "Residential Area",
            "A quiet residential street with pedestrians and parked cars"
        ),
        "parking_lot": SampleScene(
            "Parking Lot",
            "A parking lot with limited visibility and tight spaces"
        ),
    }

    return scenes.get(scene_type, SampleScene("Default Scene", "A default empty scene"))


if __name__ == "__main__":
    # Example 1: Create a single vehicle object
    print("=" * 60)
    print("EXAMPLE 1: Single Vehicle Object")
    print("=" * 60)
    vehicle = {
        "id": 1,
        "type": "auto_rickshaw",
        "x": 12.5,
        "y": 3.2,
        "speed": 5.4,
        "heading": 20
    }
    print(vehicle)
    print()

    # Example 2: Create multiple objects (standardized format)
    # This format is consistent across the entire pipeline:
    # RoadRunner/Sensors → Perception → Prediction → Planner
    print("=" * 60)
    print("EXAMPLE 2: Multiple Objects (Standardized Format)")
    print("=" * 60)
    objects = [
        {
            "id": 1,
            "type": "car",
            "x": 20,
            "y": 2,
            "speed": 8
        },
        {
            "id": 2,
            "type": "pedestrian",
            "x": 14,
            "y": -1,
            "speed": 1.2
        },
        {
            "id": 3,
            "type": "cow",
            "x": 25,
            "y": 1,
            "speed": 0.8
        }
    ]

    for obj in objects:
        print(obj)
    print()

    # Example 3: Scene creation
    print("=" * 60)
    print("EXAMPLE 3: Scene Creation")
    print("=" * 60)
    scene = create_sample_scene("urban_intersection")
    print(f"Created scene: {scene.get_scene_info()}")
