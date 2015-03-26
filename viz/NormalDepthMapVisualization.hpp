#ifndef vizkit3d_normal_depth_map_NormalDepthMapVisualization_H
#define vizkit3d_normal_depth_map_NormalDepthMapVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <>

namespace vizkit3d
{
    class NormalDepthMapVisualization
        : public vizkit3d::Vizkit3DPlugin<base::samples::RigidBodyState>
        , boost::noncopyable
    {
    Q_OBJECT
    public:
        NormalDepthMapVisualization();
        ~NormalDepthMapVisualization();

    Q_INVOKABLE void updateData(base::samples::RigidBodyState const &sample)
    {vizkit3d::Vizkit3DPlugin<base::samples::RigidBodyState>::updateData(sample);}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(base::samples::RigidBodyState const& plan);
        
    private:
        struct Data;
        Data* p;
    };
}
#endif
