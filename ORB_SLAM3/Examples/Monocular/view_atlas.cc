/**
* ORB-SLAM3 Atlas viewer (no input images required).
*/

#include <atomic>
#include <csignal>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "System.h"
#include "Atlas.h"
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"

using namespace std;

namespace {
    atomic<bool> g_shouldExit(false);

    void SignalHandler(int) {
        g_shouldExit.store(true);
    }

    void ExportMapPointsPly(ORB_SLAM3::Atlas* atlas, const string& path)
    {
        vector<ORB_SLAM3::Map*> maps = atlas->GetAllMaps();
        vector<Eigen::Vector3f> points;
        points.reserve(100000);

        for (ORB_SLAM3::Map* map : maps)
        {
            if (!map || map->IsBad())
            {
                continue;
            }
            vector<ORB_SLAM3::MapPoint*> mps = map->GetAllMapPoints();
            for (ORB_SLAM3::MapPoint* mp : mps)
            {
                if (!mp || mp->isBad())
                {
                    continue;
                }
                points.push_back(mp->GetWorldPos());
            }
        }

        ofstream out(path);
        out << "ply\nformat ascii 1.0\n";
        out << "element vertex " << points.size() << "\n";
        out << "property float x\nproperty float y\nproperty float z\n";
        out << "end_header\n";
        out << fixed << setprecision(6);
        for (const auto& p : points)
        {
            out << p.x() << " " << p.y() << " " << p.z() << "\n";
        }
    }

    void ExportKeyFramesCsv(ORB_SLAM3::Atlas* atlas, const string& path)
    {
        vector<ORB_SLAM3::Map*> maps = atlas->GetAllMaps();
        ofstream out(path);
        out << "timestamp,tx,ty,tz,qx,qy,qz,qw,kf_id,map_id\n";
        out << fixed << setprecision(6);

        for (ORB_SLAM3::Map* map : maps)
        {
            if (!map || map->IsBad())
            {
                continue;
            }
            vector<ORB_SLAM3::KeyFrame*> kfs = map->GetAllKeyFrames();
            for (ORB_SLAM3::KeyFrame* kf : kfs)
            {
                if (!kf || kf->isBad())
                {
                    continue;
                }
                Sophus::SE3f Twc = kf->GetPoseInverse();
                Eigen::Vector3f t = Twc.translation();
                Eigen::Quaternionf q(Twc.rotationMatrix());
                out << kf->mTimeStamp << ","
                    << t.x() << "," << t.y() << "," << t.z() << ","
                    << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
                    << kf->mnId << "," << map->GetId() << "\n";
            }
        }
    }
}

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        cerr << "Usage: ./view_atlas path_to_vocabulary path_to_settings "
             << "[--export_ply path] [--export_kf path] [--no_viewer]" << endl;
        return 1;
    }

    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    string plyPath;
    string kfPath;
    bool useViewer = true;

    for (int i = 3; i < argc; ++i)
    {
        string arg(argv[i]);
        if (arg == "--export_ply" && i + 1 < argc)
        {
            plyPath = argv[++i];
        }
        else if (arg == "--export_kf" && i + 1 < argc)
        {
            kfPath = argv[++i];
        }
        else if (arg == "--no_viewer")
        {
            useViewer = false;
        }
    }

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, useViewer);

    ORB_SLAM3::Atlas* atlas = SLAM.GetAtlas();
    if (atlas)
    {
        if (!plyPath.empty())
        {
            ExportMapPointsPly(atlas, plyPath);
            cout << "Exported map points to " << plyPath << endl;
        }
        if (!kfPath.empty())
        {
            ExportKeyFramesCsv(atlas, kfPath);
            cout << "Exported keyframes to " << kfPath << endl;
        }
    }

    while(!g_shouldExit.load())
    {
        this_thread::sleep_for(chrono::milliseconds(50));
    }

    SLAM.Shutdown();
    return 0;
}
