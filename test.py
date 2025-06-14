import xml.etree.ElementTree as ET

def parse_urdf_joints(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    joints = []
    for joint in root.findall("joint"):
        name = joint.attrib.get("name", "unknown")
        joint_type = joint.attrib.get("type", "unknown")
        joints.append((name, joint_type))
    return joints

def compare_joint_lists(joints1, joints2, name1="URDF1", name2="URDF2"):
    names1 = [j[0] for j in joints1]
    names2 = [j[0] for j in joints2]

    print(f"\n✅ {name1} joints ({len(joints1)}):")
    for name, joint_type in joints1:
        print(f"  - {name} ({joint_type})")

    print(f"\n✅ {name2} joints ({len(joints2)}):")
    for name, joint_type in joints2:
        print(f"  - {name} ({joint_type})")

    print("\n🔍 Comparison:")
    if names1 == names2:
        print("✅ Joint names and order are exactly the same.")
    else:
        only_in_1 = [name for name in names1 if name not in names2]
        only_in_2 = [name for name in names2 if name not in names1]

        if only_in_1:
            print(f"❌ Joints only in {name1}: {only_in_1}")
        if only_in_2:
            print(f"❌ Joints only in {name2}: {only_in_2}")

        # Check length and order
        if len(names1) != len(names2):
            print("❌ Number of joints is different.")
        else:
            print("⚠️ Joint counts match, but order might differ.")

# ========= 修改以下路径 =========
tita_urdf = "resources/tita/urdf/tita_description.urdf"
wheel_leg_urdf = "resources/wheel_leg_v2/urdf/wheel_leg_v2.urdf"

# ========= 执行比较 =========
tita_joints = parse_urdf_joints(tita_urdf)
wheel_leg_joints = parse_urdf_joints(wheel_leg_urdf)
compare_joint_lists(tita_joints, wheel_leg_joints, name1="TITA", name2="WHEEL_LEG")
