class SLAMReader:
    def __init__(self, mode="hardcoded"):
        self.mode = mode
        self.block_positions = {}  # { "block1": (x, y, z), ... }

    def get_block_positions(self):
        if self.mode == "hardcoded":
            return self._hardcoded_positions()
        return self.block_positions

    def update(self, slam_data: dict):

        # Call this when receive live SLAM data.
        
        self.block_positions = slam_data

    def _hardcoded_positions(self):
        return {
            "block1": (0.32, 0.15, 0.0),
            "block2": (0.55, 0.30, 0.0),
            "block3": (0.21, 0.44, 0.0),
            "block4": (0.60, 0.10, 0.0),
        }


# test
if __name__ == "__main__":
    reader = SLAMReader(mode="hardcoded")
    positions = reader.get_block_positions()
    print("Block positions from SLAM:")
    for name, pos in positions.items():
        print(f"  {name}: x={pos[0]}, y={pos[1]}, z={pos[2]}")
