#ifndef IMUINITIALIZER_H
#define IMUINITIALIZER_H

namespace ORB_SLAM3
{
class Atlas;

class IMUInitializer
{
public:
    explicit IMUInitializer(const Atlas* pAtlas);

    bool IsImuInitialized() const;
    bool CanStartInitialization(int minKeyFrames) const;

private:
    const Atlas* mpAtlas;
};
} // namespace ORB_SLAM3

#endif // IMUINITIALIZER_H
