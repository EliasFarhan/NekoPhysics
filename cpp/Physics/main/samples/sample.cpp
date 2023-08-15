#include "sample.h"


#include "01_kinematic.h"
#include "02_planets.h"
#include "03_triggers.h"
#include "04_ground.h"
#include "05_collisions.h"

#include <imgui.h>


namespace neko
{
SampleManager::SampleManager()
{
    samples_.emplace_back("01 Kinematic", std::make_unique<KinematicSample>());
    samples_.emplace_back("02 Planets", std::make_unique<PlanetSample>());
    samples_.emplace_back("03 Triggers", std::make_unique<TriggersSample>());
    samples_.emplace_back("04 Ground", std::make_unique<GroundSample>());
    samples_.emplace_back("05 Collisions", std::make_unique<CollisionsSample>());
}

void SampleManager::Update(float dt)
{
    fixedDt += dt;
    while (fixedDt > float{ Sample::fixedDeltaTime })
    {
        if (sampleIndex_ != INVALID_INDEX)
        {
            samples_[sampleIndex_].second->FixedUpdate();
        }
        fixedDt -= float{ Sample::fixedDeltaTime };
    }
    if (sampleIndex_ != INVALID_INDEX)
    {
        samples_[sampleIndex_].second->Update(dt);
    }

}

void SampleManager::Draw(SDL_Renderer* renderer)
{
    if (sampleIndex_ != INVALID_INDEX)
    {
        samples_[sampleIndex_].second->Draw(renderer);
    }
}

void SampleManager::DrawImGui()
{
    ImGui::Begin("Sample Manager");

    if(ImGui::BeginCombo("Samples", sampleIndex_ == INVALID_INDEX?"None":
        samples_[sampleIndex_].first.data()))
    {
        if(ImGui::Selectable("None", sampleIndex_ == INVALID_INDEX))
        {
            if(sampleIndex_ != INVALID_INDEX)
            {
                samples_[sampleIndex_].second->End();
            }
            sampleIndex_ = INVALID_INDEX;
        }
        for(std::size_t index = 0; index < samples_.size(); index++)
        {
            if(ImGui::Selectable(samples_[index].first.data(), sampleIndex_ == index))
            {
                if (sampleIndex_ != INVALID_INDEX)
                {
                    samples_[sampleIndex_].second->End();
                }
                sampleIndex_ = index;
                samples_[sampleIndex_].second->Begin();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::End();
    if(sampleIndex_ != INVALID_INDEX)
    {
        samples_[sampleIndex_].second->DrawImGui();
    }
}
} // namespace neko
