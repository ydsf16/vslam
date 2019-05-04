// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <mappoint.h>

namespace vslam
{
namespace ba
{

MapPoint::MapPoint ( const Eigen::Vector3d& position, int id, bool fixed ) : 
position_ ( position ), id_ ( id ), fixed_ ( fixed ), state_index_(-1)
{
} // MapPoint

const Eigen::Vector3d& MapPoint::getPosition()
{
    return position_;
} // getPosition

void MapPoint::setPosition ( const Eigen::Vector3d& position )
{
    position_ = position;
} // setPosition

int MapPoint::getId()
{
    return id_;
} // getId

void MapPoint::setId ( int id )
{
    id_ = id;
} // setId

void MapPoint::setFixed()
{
    fixed_ = true;
} // setFixed

bool MapPoint::isFixed()
{
	return fixed_;
} // isFixed

void MapPoint::addDeltaPosition ( const Eigen::Vector3d& delta_position )
{
    position_ += delta_position;
} // addDeltaPosition


} // namespace ba
} // namespace vslam
