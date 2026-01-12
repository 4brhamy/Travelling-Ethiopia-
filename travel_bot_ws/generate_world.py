import os

# Define the full map from Figure 5 with estimated (X, Y) coordinates
# Addis Ababa is at (0, 0)
cities = {
    "Addis_Ababa": (0, 0),
    "Debre_Birhan": (2, 5),
    "Ambo": (-4, 1),
    "Adama": (4, -2),
    "Nekemte": (-8, 2),
    "Gimbi": (-11, 2),
    "Dembi_Dollo": (-14, 1),
    "Gambella": (-15, 0),
    "Wolkite": (-4, -2),
    "Jimma": (-6, -5),
    "Bedelle": (-8, -3),
    "Gore": (-10, -4),
    "Tepi": (-12, -6),
    "Mezan_Teferi": (-12, -8),
    "Bonga": (-10, -7),
    "Dawro": (-7, -9),
    "Wolaita_Sodo": (-4, -9),
    "Arba_Minch": (-4, -12),
    "Buta_Jirra": (0, -4),
    "Batu": (2, -4),
    "Worabe": (-2, -6),
    "Hossana": (-2, -8),
    "Shashemene": (0, -9),
    "Hawassa": (0, -11),
    "Dilla": (0, -13),
    "Assella": (5, -5),
    "Assasa": (5, -8),
    "Dodolla": (4, -10),
    "Bale": (6, -10),
    "Goba": (8, -9),
    "Sof_Oumer": (10, -9),
    "Matahara": (6, -2),
    "Awash": (8, -1),
    "Chiro": (10, 0),
    "Dire_Dawa": (12, 1),
    "Harar": (13, 1),
    "Babile": (14, 1),
    "Jigjiga": (16, 1),
    "Dega_Habur": (16, -3),
    "Kebri_Dehar": (18, -6),
    "Gode": (18, -9),
}

sdf_template_header = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="ethiopia_full_world">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
"""

sdf_template_footer = """
  </world>
</sdf>
"""

def create_model_xml(name, x, y):
    # Colors: Addis=Blue, Others=Red
    color = "0 0 1 1" if name == "Addis_Ababa" else "1 0 0 1"
    
    return f"""
    <model name="{name}">
      <pose>{x} {y} 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder radius="0.5" length="1.0"/></geometry>
          <material><ambient>{color}</ambient></material>
        </visual>
        <collision name="collision">
          <geometry><cylinder radius="0.5" length="1.0"/></geometry>
        </collision>
      </link>
    </model>
    """

# Generate the file content
content = sdf_template_header
for city, (x, y) in cities.items():
    content += create_model_xml(city, x, y)
content += sdf_template_footer

# Write to the correct path
output_path = os.path.expanduser("~/travel_bot_ws/src/ethiopia_travel/worlds/ethiopia.world")
with open(output_path, "w") as f:
    f.write(content)

print(f"Successfully generated full world file at: {output_path}")