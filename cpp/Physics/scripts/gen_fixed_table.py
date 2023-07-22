import math


def gen_sqrt_tables(min_value: float, max_value: float, resolution: float):
    resolution = float(int(resolution*(1 << 16)))/(1<<16)
    results = []
    current_value = min_value
    while current_value <= max_value:
        results.append(math.sqrt(current_value))
        current_value += resolution
    with open('src/sqrt_tables.cpp', 'w') as f:
        f.write("""int sqrt_table[] = {{ {0} }};
        """
                .format(",".join(str(int(x * (1 << 16))) for x in results)))
    with open('src/sqrt_const.h', 'w') as f:
        f.write("""#pragma once\nnamespace neko\n{{
        constexpr int sqrt_min = {0}; constexpr int sqrt_max = {1}; 
        constexpr int sqrt_len = {2}; constexpr int sqrt_resolution = {3};
        }}""".format(int(min_value*(1<<16)),
                    int(max_value*(1<<16)),
                    len(results),
                    int(resolution*(1<<16))))


def gen_cos_tables(resolution: float):
    results = []
    pi_fixed = int(math.pi * (1 << 16))
    resolution = float(int(resolution*(1 << 16)))/(1 << 16)
    current_value = 0
    while current_value <= 2.0*math.pi:
        results.append(math.cos(current_value))
        current_value += resolution
    with open('src/cos_tables.cpp', 'w') as f:
        f.write("""int cos_table[] = {{ {0} }};"""
                .format(",".join(str(int(x * (1 << 16))) for x in results)))
    with open('src/cos_const.h', 'w') as f:
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
    with open('src/sin_tables.cpp', 'w') as f:
        f.write("""int sin_table[] = {{ {0} }};"""
                .format(",".join(str(int(x * (1 << 16))) for x in results)))
    with open('src/sin_const.h', 'w') as f:
        f.write("""#pragma once\n namespace neko {{
        constexpr int sin_resolution = {}; constexpr int sin_len = {};}}"""
                .format(int(resolution*(1<<16)),
                        len(results)))

def main():
    gen_sqrt_tables(0.0, 100.0, 0.0125)
    gen_cos_tables(0.0125)
    gen_sin_tables(0.0125)


if __name__ == '__main__':
    main()
