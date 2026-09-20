import argparse
import math
from typing import Dict, Optional, Tuple

class ArmKinematics:
    def __init__(self, l1: float = 150.0, l2: float = 130.0, l3: float = 50.0):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.max_reach = l1 + l2 + l3
        self.min_reach = abs(l1 - l2)

    def forward_kinematics(self, theta1_deg: float, theta2_deg: float, theta3_deg: float = 0.0) -> Tuple[float, float, float]:
        t1 = math.radians(theta1_deg)
        t2 = math.radians(theta2_deg)
        t3 = math.radians(theta3_deg)
        
        x = self.l1 * math.cos(t1) + self.l2 * math.cos(t1 + t2) + self.l3 * math.cos(t1 + t2 + t3)
        y = self.l1 * math.sin(t1) + self.l2 * math.sin(t1 + t2) + self.l3 * math.sin(t1 + t2 + t3)
        phi = math.degrees(t1 + t2 + t3)
        return round(x, 3), round(y, 3), round(phi, 2)

    def inverse_kinematics(self, target_x: float, target_y: float, target_phi_deg: float = 0.0) -> Optional[Dict]:
        phi_rad = math.radians(target_phi_deg)
        wx = target_x - self.l3 * math.cos(phi_rad)
        wy = target_y - self.l3 * math.sin(phi_rad)
        
        dist_sq = wx**2 + wy**2
        d = (dist_sq - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        
        if abs(d) > 1.0:
            return None # Out of reachable workspace
            
        # Elbow-up solution
        theta2_rad = math.atan2(math.sqrt(1.0 - d**2), d)
        theta1_rad = math.atan2(wy, wx) - math.atan2(self.l2 * math.sin(theta2_rad), self.l1 + self.l2 * math.cos(theta2_rad))
        theta3_rad = phi_rad - theta1_rad - theta2_rad
        
        t1_deg = round(math.degrees(theta1_rad), 2)
        t2_deg = round(math.degrees(theta2_rad), 2)
        t3_deg = round(math.degrees(theta3_rad), 2)
        
        # Verify with forward kinematics
        fx, fy, fphi = self.forward_kinematics(t1_deg, t2_deg, t3_deg)
        error = round(math.sqrt((fx - target_x)**2 + (fy - target_y)**2), 4)
        
        return {
            "reachable": True,
            "theta1_deg": t1_deg,
            "theta2_deg": t2_deg,
            "theta3_deg": t3_deg,
            "error_mm": error
        }

def main():
    parser = argparse.ArgumentParser(description="Robotic Arm Inverse Kinematics Solver")
    parser.add_argument("--x", type=float, default=180.0, help="Target X in mm")
    parser.add_argument("--y", type=float, default=120.0, help="Target Y in mm")
    parser.add_argument("--phi", type=float, default=0.0, help="Target Tool Angle in degrees")
    args = parser.parse_args()

    arm = ArmKinematics()
    print(f"[*] Solving Inverse Kinematics for target (X={args.x} mm, Y={args.y} mm, Phi={args.phi} deg)...")
    sol = arm.inverse_kinematics(args.x, args.y, args.phi)

    print("-" * 55)
    if sol:
        print(f"  Status        : REACHABLE")
        print(f"  Base (Theta 1): {sol['theta1_deg']} deg")
        print(f"  Elbow(Theta 2): {sol['theta2_deg']} deg")
        print(f"  Wrist(Theta 3): {sol['theta3_deg']} deg")
        print(f"  Position Error: {sol['error_mm']} mm")
    else:
        print("  Status        : UNREACHABLE (Coordinates exceed arm geometry)")
    print("-" * 55)

if __name__ == "__main__":
    main()
