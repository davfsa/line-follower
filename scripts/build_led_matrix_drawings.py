import pathlib

# Constants
REGISTER_ADDRESSES = [0x100, 0x200, 0x300, 0x400, 0x500, 0x600, 0x700, 0x800]
MATRIX_SIZE = len(REGISTER_ADDRESSES)
MATRIX_OFF_CHAR = "0"
MATRIX_ON_CHAR = "1"
VALID_CHARS = {MATRIX_OFF_CHAR, MATRIX_ON_CHAR}


def build_matrix_hex(file: pathlib.Path) -> str:
    # Extract data from file
    with file.open() as fp:
        lines = [line.strip().replace(" ", "") for line in fp.readlines()]

    # Treat extracted data
    extracted_matrix = [[0 for _ in range(MATRIX_SIZE)] for _ in range(MATRIX_SIZE)]
    for i, line in enumerate(lines):
        line_num = i + 1

        if len(line) != MATRIX_SIZE:
            print(f"Error: line {line_num}: incorrect length (expected {MATRIX_SIZE} characters, found {len(line)})")
            exit(1)

        for j, char in enumerate(line):
            if char not in VALID_CHARS:
                print(f"Error: line {line_num}: invalid character found: {char!r}")
                exit(1)

            extracted_matrix[i][j] = char

    # Build output
    output = "0x"
    for j in range(MATRIX_SIZE):
        data = 0
        for i in range(MATRIX_SIZE):
            char = extracted_matrix[i][j]

            if char == MATRIX_ON_CHAR:
                data |= 1 << i

        output += f"{hex(data)[2:]:>02}"

    return output


for drawing in sorted(pathlib.Path("matrix_drawings").glob("*.txt")):
    print(f"{drawing.name}: {build_matrix_hex(drawing)}")
