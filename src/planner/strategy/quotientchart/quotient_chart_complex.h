#pragma once
#include "quotient_chart.h"
#include <gudhi/Simplex_tree.h>

namespace ob = ompl::base;

namespace ompl
{
  namespace geometric
  {
    class QuotientChartComplex: public QuotientChart
    {
        typedef og::QuotientChart BaseT;
        typedef og::QuotientGraph::Graph Graph;
        using Simplex_tree = Gudhi::Simplex_tree<>;

      public:
        QuotientChartComplex(const ob::SpaceInformationPtr &si, Quotient *parent_ = nullptr);
        virtual void setup() override;

        virtual double Distance(const Vertex a, const Vertex b) const override;
        virtual void Grow(double t) override;
        virtual bool Sample(ob::State *q_random) override;
        //virtual void growRoadmap(const ob::PlannerTerminationCondition &ptc, ob::State *) override;
        virtual void getPlannerData(ob::PlannerData &data) const override;

        void AddSimplices(Vertex v1, Vertex v2);
        void RemoveSimplices(Vertex v1, Vertex v2);

      private:
        double epsilon_max_neighborhood{1.0};
        Simplex_tree simplex;

        uint ntry;

        struct Ksimplex{
          Ksimplex(std::vector<int> vertices_):
            vertices(vertices_) 
          {
            uint N = vertices.size();
            if(N>2) comb(N,N-1);
          };

          void comb(int N, int K)
          {
            std::string bitmask(K, 1); // K leading 1's
            bitmask.resize(N, 0); // N-K trailing 0's
            do {
              std::vector<int> facet;
              for (int i = 0; i < N; i++)
              {
                  if (bitmask[i]) facet.push_back(vertices.at(i));
              }
              Ksimplex *kfacet = simplicial_complex[facet];
              facets.push_back(kfacet);
              kfacet->AddCoFace(this);

            } while (std::prev_permutation(bitmask.begin(), bitmask.end()));
          }

          void Clear()
          {
            for(uint k = 0; k < cofaces.size(); k++){
              cofaces.at(k)->Clear();
            }
            for(uint k = 0; k < facets.size(); k++){
              Ksimplex *facet = facets.at(k);
              for(uint j = 0; j < facet->cofaces.size(); j++){
                Ksimplex *coface = facet->cofaces.at(j);
                if(coface == this)
                {
                  facet->cofaces.erase(facet->cofaces.begin() + j);
                  break;
                }
              }
            }
            //no pointers should be left, we can remove this Ksimplex
          }

          void AddCoFace(Ksimplex* coface)
          {
            cofaces.push_back(coface);
          }

          std::vector<Ksimplex*> facets;
          std::vector<Ksimplex*> cofaces;
          std::vector<int> vertices;
        };

        std::map<std::vector<int>, Ksimplex*> simplicial_complex;

        //typedef std::vector< std::vector<int> > LocalSimplicialComplex;
        //typedef std::pair<const ob::State*, const ob::State*> EdgeVertices;
        //std::map<const ob::State*, LocalSimplicialComplex> simplicial_complex;
        //std::map<Edge, LocalSimplicialComplex> simplicial_complex;

        RoadmapNeighbors nn_infeasible;
    };
  }
}
