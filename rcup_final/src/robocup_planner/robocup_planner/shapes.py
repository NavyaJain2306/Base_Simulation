#  Available target shapes 

SHAPES = {

    # 2x2 square
    "square_2x2": {
        "pos_A": (0.50, 0.50, 0.0),
        "pos_B": (0.50, 0.60, 0.0),
        "pos_C": (0.60, 0.50, 0.0),
        "pos_D": (0.60, 0.60, 0.0),
    },

    # L-shape (3 blocks)
    "L_shape": {
        "pos_A": (0.50, 0.50, 0.0),
        "pos_B": (0.50, 0.60, 0.0),
        "pos_C": (0.50, 0.70, 0.0),
        "pos_D": (0.60, 0.70, 0.0),
    },

    # Straight line (4 blocks)
    "line_4": {
        "pos_A": (0.50, 0.40, 0.0),
        "pos_B": (0.50, 0.50, 0.0),
        "pos_C": (0.50, 0.60, 0.0),
        "pos_D": (0.50, 0.70, 0.0),
    },

    # T-shape (4 blocks)
    "T_shape": {
        "pos_A": (0.50, 0.50, 0.0),
        "pos_B": (0.60, 0.50, 0.0),
        "pos_C": (0.70, 0.50, 0.0),
        "pos_D": (0.60, 0.60, 0.0),
    },
}


def get_shape(name: str) -> dict:
    if name not in SHAPES:
        raise ValueError(f"Unknown shape '{name}'. Available: {list(SHAPES.keys())}")
    return SHAPES[name]


#  Quick test 
if __name__ == "__main__":
    for shape_name, positions in SHAPES.items():
        print(f"\n{shape_name}:")
        for pos_name, coords in positions.items():
            print(f"  {pos_name}: {coords}")
