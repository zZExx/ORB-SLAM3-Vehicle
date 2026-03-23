#include "IMUInitializer.h"

#include "Atlas.h"

namespace ORB_SLAM3
{
IMUInitializer::IMUInitializer(const Atlas* pAtlas)
    : mpAtlas(pAtlas)
{
}

bool IMUInitializer::IsImuInitialized() const
{
    Atlas* pAtlas = const_cast<Atlas*>(mpAtlas);
    return pAtlas != nullptr && pAtlas->isImuInitialized();
}

bool IMUInitializer::CanStartInitialization(int minKeyFrames) const
{
    Atlas* pAtlas = const_cast<Atlas*>(mpAtlas);
    return pAtlas != nullptr && pAtlas->KeyFramesInMap() >= static_cast<unsigned long>(minKeyFrames) && !pAtlas->isImuInitialized();
}
} // namespace ORB_SLAM3
