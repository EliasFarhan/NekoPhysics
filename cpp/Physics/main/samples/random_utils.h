#pragma once

#include <type_traits>
#include <random>


template<typename T>
typename std::enable_if<std::is_integral<T>::value, T>::type RandomRange(T start, T end)
{
    static std::random_device rd;  //Will be used to obtain a seed for the random number engine
    static std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<T> dis(start, end);
    return dis(gen);
}

template<typename T>
typename std::enable_if<std::is_floating_point<T>::value, T>::type RandomRange(T start, T end)
{
    static std::random_device rd;  //Will be used to obtain a seed for the random number engine
    static std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<T> dis(start, end);
    return dis(gen);
}

template<typename T, typename U>
void RandomFill(T& container, U low, U high)
{
    for(auto& elem: container)
    {
        elem = RandomRange(low, high);
    }
}