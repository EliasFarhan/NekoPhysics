#pragma once

#include <memory>
#include <SDL_render.h>
#include <string>
#include <vector>

#include "math/const.h"

namespace neko
{

class Sample
{
public:
    virtual ~Sample() = default;
    virtual void Begin() = 0;
    virtual void Update(float dt) = 0;
    virtual void FixedUpdate(){}
    virtual void Draw(SDL_Renderer* renderer) = 0;
    virtual void End() = 0;
    virtual void DrawImGui(){}

    static constexpr auto fixedDeltaTime = Scalar{ 1.0f / 60.0f };
};

class SampleManager
{
public:
    SampleManager();
    void Update(float dt);
    void Draw(SDL_Renderer* renderer);
    void DrawImGui();
private:
    std::vector<std::pair<std::string, std::unique_ptr<Sample>>> samples_;
    static constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();
    std::size_t sampleIndex_ = INVALID_INDEX;
	float fixedDt = 0.0f;
};

}
