import math
import os


def gen_cos_tables(resolution: float):
    results = []
    pi_fixed = int(math.pi * (1 << 16))
    resolution = float(int(resolution*(1 << 16)))/(1 << 16)
    current_value = 0
    while current_value <= 2.0*math.pi:
        results.append(math.cos(current_value))
        current_value += resolution
    with open('src/generated/cos_tables.cpp', 'w') as f:
        f.write("""int cos_table[] = {{ {0} }};"""
                .format(",".join(str(int(x * (1 << 16))) for x in results)))
    with open('src/generated/cos_const.h', 'w') as f:
        f.write("""#pragma once\n namespace neko {{
        constexpr int cos_resolution = {}; constexpr int cos_len = {}; constexpr int fixed_pi = {};}}"""
                .format(int(resolution*(1<<16)),
                        len(results),
                        pi_fixed))


def gen_sin_tables(resolution: float):
    results = []
    resolution = float(int(resolution*(1 << 16)))/(1<<16)
    current_value = 0
    while current_value <= 2.0*math.pi:
        results.append(math.sin(current_value))
        current_value += resolution
    with open('src/generated/sin_tables.cpp', 'w') as f:
        f.write("""int sin_table[] = {{ {0} }};"""
                .format(",".join(str(int(x * (1 << 16))) for x in results)))
    with open('src/generated/sin_const.h', 'w') as f:
        f.write("""#pragma once\n namespace neko {{
        constexpr int sin_resolution = {}; constexpr int sin_len = {};}}"""
                .format(int(resolution*(1<<16)),
                        len(results)))

def main():
    if not os.path.isdir('src/generated'):
        os.mkdir("src/generated")
    gen_cos_tables(0.0075)
    gen_sin_tables(0.0075)


if __name__ == '__main__':
    main()
