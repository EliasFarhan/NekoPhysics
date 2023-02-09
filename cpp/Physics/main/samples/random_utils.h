#pragma once

#include <type_traits>
#include <random>

template<typename T>
T RandomRange(T low, T high) requires std::is_integral_v<T>
{
    static std::random_device rd;  //Will be used to obtain a seed for the random number engine
    static std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<T> dis(low, high);
    return dis(gen);
}

template<typename T>
T RandomRange(T low, T high) requires std::is_floating_point_v<T>
{
    static std::random_device rd;  //Will be used to obtain a seed for the random number engine
    static std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<T> dis(low, high);
    return dis(gen);
}

template<typename T, typename U>
requires std::ranges::range<T>
void RandomFill(T& container, U low, U high)
{
    for(auto& elem: container)
    {
        elem = RandomRange(low, high);
    }
}