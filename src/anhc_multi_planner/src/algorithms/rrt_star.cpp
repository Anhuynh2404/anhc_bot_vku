// Copyright 2026 An Huynh <anhuynh@anhc.dev>
// SPDX-License-Identifier: Apache-2.0
//
// rrt_star.cpp — RRT* (Rapidly-exploring Random Tree, asymptotically optimal)

#include <cmath>
#include <limits>
#include <random>
#include <vector>
#include <functional>

#include "anhc_multi_planner/algorithm_base.hpp"

namespace anhc_multi_planner
{

struct RrtNode { int x, y, parent; double cost; };

static int rrNearest(const std::vector<RrtNode>& nodes, int qx, int qy)
{
  int best = 0; double bd = std::numeric_limits<double>::infinity();
  for (int i = 0; i < (int)nodes.size(); ++i) {
    double d = cellDist(nodes[i].x, nodes[i].y, qx, qy);
    if (d < bd) { bd = d; best = i; }
  }
  return best;
}

static std::pair<int,int> rrSteer(int fx, int fy, int tx, int ty, double step)
{
  double dx = tx-fx, dy = ty-fy, dist = std::hypot(dx, dy);
  if (dist <= step) return {tx, ty};
  return {(int)std::round(fx + dx/dist*step), (int)std::round(fy + dy/dist*step)};
}

static bool rrSegFree(int x0,int y0,int x1,int y1,int W,int H,
  nav2_costmap_2d::Costmap2D* cm, bool allow_unk)
{
  int dx=std::abs(x1-x0), dy=std::abs(y1-y0);
  int sx=(x0<x1)?1:-1, sy=(y0<y1)?1:-1, err=dx-dy, cx=x0, cy=y0;
  while (true) {
    if (cx<0||cx>=W||cy<0||cy>=H) return false;
    if (!isTraversable(cx,cy,cm,allow_unk)) return false;
    if (cx==x1&&cy==y1) return true;
    int e2=2*err;
    if (e2>-dy){err-=dy;cx+=sx;}
    if (e2< dx){err+=dx;cy+=sy;}
  }
}

CellPath runRRTStar(int sx,int sy,int gx,int gy,
  nav2_costmap_2d::Costmap2D* costmap,
  const AlgoParams& p, std::function<bool()> cancel)
{
  const int W=(int)costmap->getSizeInCellsX(), H=(int)costmap->getSizeInCellsY();
  std::mt19937 rng{42u};
  std::uniform_int_distribution<int> rx(0,W-1), ry(0,H-1);
  std::uniform_real_distribution<double> rb(0.0,1.0);

  std::vector<RrtNode> nodes;
  nodes.reserve(p.rrt_star_max_nodes+1);
  nodes.push_back({sx,sy,-1,0.0});

  int goal_node=-1; double goal_cost=std::numeric_limits<double>::infinity();

  for (int i=0; i<p.rrt_star_max_nodes; ++i) {
    if (cancel && cancel()) return {};
    int qx,qy;
    if (rb(rng)<0.10) {qx=gx;qy=gy;} else {qx=rx(rng);qy=ry(rng);}

    int ni=rrNearest(nodes,qx,qy);
    auto [nx,ny]=rrSteer(nodes[ni].x,nodes[ni].y,qx,qy,p.rrt_star_step_size);
    if (nx<0||nx>=W||ny<0||ny>=H) continue;
    if (!isTraversable(nx,ny,costmap,p.allow_unknown)) continue;
    if (!rrSegFree(nodes[ni].x,nodes[ni].y,nx,ny,W,H,costmap,p.allow_unknown)) continue;

    int bp=ni; double bc=nodes[ni].cost+cellDist(nodes[ni].x,nodes[ni].y,nx,ny);
    for (int j=0;j<(int)nodes.size();++j) {
      if (cellDist(nodes[j].x,nodes[j].y,nx,ny)>p.rrt_star_rewire_radius) continue;
      if (!rrSegFree(nodes[j].x,nodes[j].y,nx,ny,W,H,costmap,p.allow_unknown)) continue;
      double c=nodes[j].cost+cellDist(nodes[j].x,nodes[j].y,nx,ny);
      if (c<bc){bc=c;bp=j;}
    }
    int nidx=(int)nodes.size();
    nodes.push_back({nx,ny,bp,bc});

    for (int j=0;j<nidx;++j) {
      if (cellDist(nx,ny,nodes[j].x,nodes[j].y)>p.rrt_star_rewire_radius) continue;
      if (!rrSegFree(nx,ny,nodes[j].x,nodes[j].y,W,H,costmap,p.allow_unknown)) continue;
      double c=bc+cellDist(nx,ny,nodes[j].x,nodes[j].y);
      if (c<nodes[j].cost){nodes[j].parent=nidx;nodes[j].cost=c;}
    }
    if (cellDist(nx,ny,gx,gy)<=p.rrt_star_step_size) {
      if (rrSegFree(nx,ny,gx,gy,W,H,costmap,p.allow_unknown)) {
        double gc=bc+cellDist(nx,ny,gx,gy);
        if (gc<goal_cost) {
          goal_cost=gc;
          if (goal_node==-1) {goal_node=(int)nodes.size(); nodes.push_back({gx,gy,nidx,gc});}
          else {nodes[goal_node].parent=nidx; nodes[goal_node].cost=gc;}
        }
      }
    }
  }
  if (goal_node==-1) return {};
  CellPath path;
  int idx=goal_node;
  while(idx!=-1){path.emplace_back(nodes[idx].x,nodes[idx].y);idx=nodes[idx].parent;}
  std::reverse(path.begin(),path.end());
  return path;
}

}  // namespace anhc_multi_planner
