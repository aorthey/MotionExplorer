#include "QuotientTopologySparse.h"
#include "elements/plannerdata_vertex_annotated.h"
#include <ompl/tools/config/SelfConfig.h>
#include <boost/foreach.hpp>

using namespace og;
using namespace ob;
#define foreach BOOST_FOREACH

QuotientTopologySparse::QuotientTopologySparse(const ob::SpaceInformationPtr &si, Quotient *parent_ ):
  BaseT(si, parent_)
{
    setName("QuotientTopologySparse"+std::to_string(id));
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;

    psimp_ = std::make_shared<PathSimplifier>(si_);

    // Planner::declareParam<double>("stretch_factor", this, &QuotientTopologySparse::setStretchFactor, &QuotientTopologySparse::getStretchFactor,
    //                               "1.1:0.1:3.0");
    // Planner::declareParam<double>("sparse_delta_fraction", this, &QuotientTopologySparse::setSparseDeltaFraction,
    //                               &QuotientTopologySparse::getSparseDeltaFraction, "0.0:0.01:1.0");
    // Planner::declareParam<double>("dense_delta_fraction", this, &QuotientTopologySparse::setDenseDeltaFraction,
    //                               &QuotientTopologySparse::getDenseDeltaFraction, "0.0:0.0001:0.1");
    // Planner::declareParam<unsigned int>("max_failures", this, &QuotientTopologySparse::setMaxFailures, &QuotientTopologySparse::getMaxFailures,
    //                                     "100:10:3000");

}

QuotientTopologySparse::~QuotientTopologySparse()
{
}

void QuotientTopologySparse::setup()
{
    BaseT::setup();

    double maxExt = si_->getMaximumExtent();
    sparseDelta_ = sparseDeltaFraction_ * maxExt;

}

void QuotientTopologySparse::clear()
{
    BaseT::clear();
}


void QuotientTopologySparse::Init()
{
  BaseT::Init();
}

void QuotientTopologySparse::Grow(double t)
{
    std::vector<Configuration*> graphNeighborhood;
    std::vector<Configuration*> visibleNeighborhood;

    BaseT::Grow(t);

    // Configuration *q_last = graphDense_[v_last_added];

    // findGraphNeighbors(q_last, graphNeighborhood, visibleNeighborhood);

    // if (checkAddCoverage(q_last->state, visibleNeighborhood))
    // {
    //     addGuard(si_->cloneState(q_last->state), COVERAGE);
    // }

    // {
    //     if (!checkAddConnectivity(qNew, visibleNeighborhood))
    //     {
    //         if (!checkAddInterface(qNew, graphNeighborhood, visibleNeighborhood))
    //         {
    //             if (!visibleNeighborhood.empty())
    //             {
    //                 std::map<Vertex, base::State *> closeRepresentatives;
    //                 findCloseRepresentatives(workState, qNew, visibleNeighborhood[0], closeRepresentatives, ptc);
    //                 for (auto &closeRepresentative : closeRepresentatives)
    //                 {
    //                     updatePairPoints(visibleNeighborhood[0], qNew, closeRepresentative.first,
    //                                      closeRepresentative.second);
    //                     updatePairPoints(closeRepresentative.first, closeRepresentative.second,
    //                                      visibleNeighborhood[0], qNew);
    //                 }
    //                 checkAddPath(visibleNeighborhood[0]);
    //                 for (auto &closeRepresentative : closeRepresentatives)
    //                 {
    //                     checkAddPath(closeRepresentative.first);
    //                     si_->freeState(closeRepresentative.second);
    //                 }
    //             }
    //         }
    //     }
    // }
}

bool QuotientTopologySparse::checkAddCoverage(const base::State *qNew, std::vector<Configuration*> &visibleNeighborhood)
{
  return visibleNeighborhood.empty();
    // if (!visibleNeighborhood.empty())
    //     return false;
    // // No free paths means we add for coverage
    // addGuard(si_->cloneState(qNew), COVERAGE);
    // return true;
}

// bool QuotientTopologySparse::checkAddConnectivity(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood)
// {
//     std::vector<Vertex> links;
//     if (visibleNeighborhood.size() > 1)
//     {
//         // For each neighbor
//         for (std::size_t i = 0; i < visibleNeighborhood.size(); ++i)
//             // For each other neighbor
//             for (std::size_t j = i + 1; j < visibleNeighborhood.size(); ++j)
//                 // If they are in different components
//                 if (!sameComponent(visibleNeighborhood[i], visibleNeighborhood[j]))
//                 {
//                     links.push_back(visibleNeighborhood[i]);
//                     links.push_back(visibleNeighborhood[j]);
//                 }

//         if (!links.empty())
//         {
//             // Add the node
//             Vertex g = addGuard(si_->cloneState(qNew), CONNECTIVITY);

//             // for (unsigned long link : links)
//             //     // If there's no edge
//             //     if (!boost::edge(g, link, g_).second)
//             //         // And the components haven't been united by previous links
//             //         if (!sameComponent(link, g))
//             //             connectGuards(g, link);
//             return true;
//         }
//     }
//     return false;
// }

// bool QuotientTopologySparse::checkAddInterface(const base::State *qNew, std::vector<Vertex> &graphNeighborhood,
//                                                   std::vector<Vertex> &visibleNeighborhood)
// {
//     // If we have more than 1 or 0 neighbors
//     if (visibleNeighborhood.size() > 1)
//         if (graphNeighborhood[0] == visibleNeighborhood[0] && graphNeighborhood[1] == visibleNeighborhood[1])
//             // If our two closest neighbors don't share an edge
//             if (!boost::edge(visibleNeighborhood[0], visibleNeighborhood[1], g_).second)
//             {
//                 // If they can be directly connected
//                 if (si_->checkMotion(graphSparse_[visibleNeighborhood[0]], graphSparse_[visibleNeighborhood[1]]))
//                 {
//                     // Connect them
//                     connectGuards(visibleNeighborhood[0], visibleNeighborhood[1]);
//                     // And report that we added to the roadmap
//                     resetFailures();
//                     // Report success
//                     return true;
//                 }

//                 // Add the new node to the graph, to bridge the interface
//                 Vertex v = addGuard(si_->cloneState(qNew), INTERFACE);
//                 connectGuards(v, visibleNeighborhood[0]);
//                 connectGuards(v, visibleNeighborhood[1]);
//                 // Report success
//                 return true;
//             }
//     return false;
// }

// bool QuotientTopologySparse::checkAddPath(Vertex v)
// {
//     bool ret = false;

//     std::vector<Vertex> rs;
//     foreach (Vertex r, boost::adjacent_vertices(v, g_))
//         rs.push_back(r);

//     /* Candidate x vertices as described in the method, filled by function computeX(). */
//     std::vector<Vertex> Xs;

//     /* Candidate v" vertices as described in the method, filled by function computeVPP(). */
//     std::vector<Vertex> VPPs;

//     for (std::size_t i = 0; i < rs.size() && !ret; ++i)
//     {
//         Vertex r = rs[i];
//         computeVPP(v, r, VPPs);
//         foreach (Vertex rp, VPPs)
//         {
//             // First, compute the longest path through the graph
//             computeX(v, r, rp, Xs);
//             double rm_dist = 0.0;
//             foreach (Vertex rpp, Xs)
//             {
//                 double tmp_dist = (si_->distance(graphSparse_[r], graphSparse_[v]) +
//                                    si_->distance(graphSparse_[v], graphSparse_[rpp])) /
//                                   2.0;
//                 if (tmp_dist > rm_dist)
//                     rm_dist = tmp_dist;
//             }

//             InterfaceData &d = getData(v, r, rp);

//             // Then, if the spanner property is violated
//             if (rm_dist > stretchFactor_ * d.d_)
//             {
//                 ret = true;  // Report that we added for the path
//                 if (si_->checkMotion(graphSparse_[r], graphSparse_[rp]))
//                     connectGuards(r, rp);
//                 else
//                 {
//                     auto p(std::make_shared<PathGeometric>(si_));
//                     if (r < rp)
//                     {
//                         p->append(d.sigmaA_);
//                         p->append(d.pointA_);
//                         p->append(graphSparse_[v]);
//                         p->append(d.pointB_);
//                         p->append(d.sigmaB_);
//                     }
//                     else
//                     {
//                         p->append(d.sigmaB_);
//                         p->append(d.pointB_);
//                         p->append(graphSparse_[v]);
//                         p->append(d.pointA_);
//                         p->append(d.sigmaA_);
//                     }

//                     psimp_->reduceVertices(*p, 10);
//                     psimp_->shortcutPath(*p, 50);

//                     if (p->checkAndRepair(100).second)
//                     {
//                         Vertex prior = r;
//                         Vertex vnew;
//                         std::vector<base::State *> &states = p->getStates();

//                         foreach (base::State *st, states)
//                         {
//                             // no need to clone st, since we will destroy p; we just copy the pointer
//                             vnew = addGuard(st, QUALITY);

//                             connectGuards(prior, vnew);
//                             prior = vnew;
//                         }
//                         // clear the states, so memory is not freed twice
//                         states.clear();
//                         connectGuards(prior, rp);
//                     }
//                 }
//             }
//         }
//     }

//     return ret;
// }

// void QuotientTopologySparse::resetFailures()
// {
//     consecutiveFailures_ = 0;
// }

void QuotientTopologySparse::findGraphNeighbors(Configuration *q, std::vector<Configuration*> &graphNeighborhood,
                                                   std::vector<Configuration*> &visibleNeighborhood)
{
    visibleNeighborhood.clear();
    nearestSparse_->nearestR(q, sparseDelta_, graphNeighborhood);

    for (Configuration* qn : graphNeighborhood)
        if (si_->checkMotion(q->state, qn->state))
            visibleNeighborhood.push_back(qn);
}

// void QuotientTopologySparse::approachGraph(Vertex v)
// {
//     std::vector<Vertex> hold;
//     nn_->nearestR(v, sparseDelta_, hold);

//     std::vector<Vertex> neigh;
//     for (unsigned long i : hold)
//         if (si_->checkMotion(graphSparse_[v], graphSparse_[i]))
//             neigh.push_back(i);

//     foreach (Vertex vp, neigh)
//         connectGuards(v, vp);
// }

// QuotientTopologySparse::Vertex QuotientTopologySparse::findGraphRepresentative(base::State *st)
// {
//     std::vector<Vertex> nbh;
//     graphSparse_[queryVertex_] = st;
//     nn_->nearestR(queryVertex_, sparseDelta_, nbh);
//     graphSparse_[queryVertex_] = nullptr;

//     Vertex result = boost::graph_traits<Graph>::null_vertex();

//     for (unsigned long i : nbh)
//         if (si_->checkMotion(st, graphSparse_[i]))
//         {
//             result = i;
//             break;
//         }
//     return result;
// }

// void QuotientTopologySparse::findCloseRepresentatives(base::State *workArea, const base::State *qNew,
//                                                          const Vertex qRep,
//                                                          std::map<Vertex, base::State *> &closeRepresentatives,
//                                                          const base::PlannerTerminationCondition &ptc)
// {
//     for (auto &closeRepresentative : closeRepresentatives)
//         si_->freeState(closeRepresentative.second);
//     closeRepresentatives.clear();

//     // Then, begin searching the space around him
//     for (unsigned int i = 0; i < nearSamplePoints_; ++i)
//     {
//         do
//         {
//             sampler_->sampleNear(workArea, qNew, denseDelta_);
//         } while ((!si_->isValid(workArea) || si_->distance(qNew, workArea) > denseDelta_ ||
//                   !si_->checkMotion(qNew, workArea)) &&
//                  !ptc);

//         // if we were not successful at sampling a desirable state, we are out of time
//         if (ptc)
//             break;

//         // Compute who his graph neighbors are
//         Vertex representative = findGraphRepresentative(workArea);

//         // Assuming this sample is actually seen by somebody (which he should be in all likelihood)
//         if (representative != boost::graph_traits<Graph>::null_vertex())
//         {
//             // If his representative is different than qNew
//             if (qRep != representative)
//                 // And we haven't already tracked this representative
//                 if (closeRepresentatives.find(representative) == closeRepresentatives.end())
//                     // Track the representative
//                     closeRepresentatives[representative] = si_->cloneState(workArea);
//         }
//         else
//         {
//             // This guy can't be seen by anybody, so we should take this opportunity to add him
//             addGuard(si_->cloneState(workArea), COVERAGE);

//             // We should also stop our efforts to add a dense path
//             for (auto &closeRepresentative : closeRepresentatives)
//                 si_->freeState(closeRepresentative.second);
//             closeRepresentatives.clear();
//             break;
//         }
//     }
// }

// void QuotientTopologySparse::updatePairPoints(Vertex rep, const base::State *q, Vertex r, const base::State *s)
// {
//     // First of all, we need to compute all candidate r'
//     std::vector<Vertex> VPPs;
//     computeVPP(rep, r, VPPs);

//     // Then, for each pair Pv(r,r')
//     foreach (Vertex rp, VPPs)
//         // Try updating the pair info
//         distanceCheck(rep, q, r, s, rp);
// }

// void QuotientTopologySparse::computeVPP(Vertex v, Vertex vp, std::vector<Vertex> &VPPs)
// {
//     VPPs.clear();
//     foreach (Vertex cvpp, boost::adjacent_vertices(v, g_))
//         if (cvpp != vp)
//             if (!boost::edge(cvpp, vp, g_).second)
//                 VPPs.push_back(cvpp);
// }

// void QuotientTopologySparse::computeX(Vertex v, Vertex vp, Vertex vpp, std::vector<Vertex> &Xs)
// {
//     Xs.clear();

//     foreach (Vertex cx, boost::adjacent_vertices(vpp, g_))
//         if (boost::edge(cx, v, g_).second && !boost::edge(cx, vp, g_).second)
//         {
//             InterfaceData &d = getData(v, vpp, cx);
//             if ((vpp < cx && (d.pointA_ != nullptr)) || (cx < vpp && (d.pointB_ != nullptr)))
//                 Xs.push_back(cx);
//         }
//     Xs.push_back(vpp);
// }

// QuotientTopologySparse::VertexPair QuotientTopologySparse::index(Vertex vp, Vertex vpp)
// {
//     if (vp < vpp)
//         return VertexPair(vp, vpp);
//     if (vpp < vp)
//         return VertexPair(vpp, vp);
//     else
//         throw Exception(name_, "Trying to get an index where the pairs are the same point!");
// }

// QuotientTopologySparse::InterfaceData &QuotientTopologySparse::getData(Vertex v, Vertex vp, Vertex vpp)
// {
//     return interfaceDataProperty_[v][index(vp, vpp)];
// }

// void QuotientTopologySparse::distanceCheck(Vertex rep, const base::State *q, Vertex r, const base::State *s,
//                                               Vertex rp)
// {
//     // Get the info for the current representative-neighbors pair
//     InterfaceData &d = getData(rep, r, rp);

//     if (r < rp)  // FIRST points represent r (the guy discovered through sampling)
//     {
//         if (d.pointA_ == nullptr)  // If the point we're considering replacing (P_v(r,.)) isn't there
//             // Then we know we're doing better, so add it
//             d.setFirst(q, s, si_);
//         else  // Otherwise, he is there,
//         {
//             if (d.pointB_ == nullptr)  // But if the other guy doesn't exist, we can't compare.
//             {
//                 // Should probably keep the one that is further away from rep?  Not known what to do in this case.
//                 // \todo: is this not part of the algorithm?
//             }
//             else  // We know both of these points exist, so we can check some distances
//                 if (si_->distance(q, d.pointB_) < si_->distance(d.pointA_, d.pointB_))
//                 // Distance with the new point is good, so set it.
//                 d.setFirst(q, s, si_);
//         }
//     }
//     else  // SECOND points represent r (the guy discovered through sampling)
//     {
//         if (d.pointB_ == nullptr)  // If the point we're considering replacing (P_V(.,r)) isn't there...
//             // Then we must be doing better, so add it
//             d.setSecond(q, s, si_);
//         else  // Otherwise, he is there
//         {
//             if (d.pointA_ == nullptr)  // But if the other guy doesn't exist, we can't compare.
//             {
//                 // Should we be doing something cool here?
//             }
//             else if (si_->distance(q, d.pointA_) < si_->distance(d.pointB_, d.pointA_))
//                 // Distance with the new point is good, so set it
//                 d.setSecond(q, s, si_);
//         }
//     }

//     // Lastly, save what we have discovered
//     interfaceDataProperty_[rep][index(r, rp)] = d;
// }

// void QuotientTopologySparse::abandonLists(base::State *st)
// {
//     graphSparse_[queryVertex_] = st;

//     std::vector<Vertex> hold;
//     nn_->nearestR(queryVertex_, sparseDelta_, hold);

//     graphSparse_[queryVertex_] = nullptr;

//     foreach (Vertex v, hold)
//     {
//         foreach (VertexPair r, interfaceDataProperty_[v] | boost::adaptors::map_keys)
//             interfaceDataProperty_[v][r].clear(si_);
//     }
// }

QuotientTopologySparse::Vertex QuotientTopologySparse::addGuard(base::State *state, GuardType type)
{
    Configuration* q_guard = new Configuration(Q1, state);
    Vertex v_guard = AddConfiguration(q_guard);

//     G[v_guard]->colorProperty = type;

//     abandonLists(state);

//     disjointSets_.make_set(m);
//     nn_->add(m);

    return v_guard;
}

// void QuotientTopologySparse::connectGuards(Vertex v, Vertex vp)
// {
//     const base::Cost weight(costHeuristic(v, vp));
//     const Graph::edge_property_type properties(weight);
//     std::lock_guard<std::mutex> _(graphMutex_);
//     boost::add_edge(v, vp, properties, g_);
//     disjointSets_.union_set(v, vp);
// }


// void QuotientTopologySparse::getPlannerData(base::PlannerData &data) const
// {
//     Planner::getPlannerData(data);

//     // Explicitly add start and goal states:
//     for (unsigned long i : startM_)
//         data.addStartVertex(base::PlannerDataVertex(graphSparse_[i], (int)START));

//     for (unsigned long i : goalM_)
//         data.addGoalVertex(base::PlannerDataVertex(graphSparse_[i], (int)GOAL));

//     // If there are even edges here
//     if (boost::num_edges(g_) > 0)
//     {
//         // Adding edges and all other vertices simultaneously
//         foreach (const Edge e, boost::edges(g_))
//         {
//             const Vertex v1 = boost::source(e, g_);
//             const Vertex v2 = boost::target(e, g_);
//             data.addEdge(base::PlannerDataVertex(graphSparse_[v1], (int)colorProperty_[v1]),
//                          base::PlannerDataVertex(graphSparse_[v2], (int)colorProperty_[v2]));

//             // Add the reverse edge, since we're constructing an undirected roadmap
//             data.addEdge(base::PlannerDataVertex(graphSparse_[v2], (int)colorProperty_[v2]),
//                          base::PlannerDataVertex(graphSparse_[v1], (int)colorProperty_[v1]));
//         }
//     }
//     else
//         OMPL_INFORM("%s: There are no edges in the graph!", getName().c_str());

//     // Make sure to add edge-less nodes as well
//     foreach (const Vertex n, boost::vertices(g_))
//         if (boost::out_degree(n, g_) == 0)
//             data.addVertex(base::PlannerDataVertex(graphSparse_[n], (int)colorProperty_[n]));
// }

ompl::base::Cost QuotientTopologySparse::costHeuristic(Vertex u, Vertex v) const
{
    return opt_->motionCostHeuristic(graphSparse_[u]->state, graphSparse_[v]->state);
}

