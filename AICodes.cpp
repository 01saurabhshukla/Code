BFS

#include <bits/stdc++.h>

using namespace std;
class Solution {
  public:
    vector < int > bfsOfGraph(int V, vector < int > adj[]) {
      vector < int > bfs;
      vector < int > vis(V, 0);
      queue < int > q;
      q.push(0);
      vis[0] = 1;
      while (!q.empty()) {
        int node = q.front();
        q.pop();
        bfs.push_back(node);

        for (auto it: adj[node]) {
          if (!vis[it]) {
            q.push(it);
            vis[it] = 1;
          }
        }
      }

      return bfs;
    }
};

void addEdge(vector < int > adj[], int u, int v) {
  adj[u].push_back(v);
  adj[v].push_back(u);
}

void printAns(vector < int > & ans) {
  for (int i = 0; i < ans.size(); i++) {
    cout << ans[i] << " ";
  }
}
int main() {
  vector < int > adj[5];

  addEdge(adj, 0, 1);
  addEdge(adj, 0, 2);
  addEdge(adj, 0, 3);
  addEdge(adj, 2, 4);

  Solution obj;
  vector < int > ans = obj.bfsOfGraph(5, adj);
  printAns(ans);
  cout << endl;

  return 0;
}



DFS
#include<bits/stdc++.h>
using namespace std;

class solution {
    void dfs(int  node,vector<int > adj[],vector<int> &vis,vector<int> &storedfs){
        vis[node]=1;
        storedfs.push_back(node);

        for(auto it : adj[node]){
            if(!vis[it]){
                dfs(it, adj, vis, storedfs);
            }
        }

        
    }

    public:
     vector<int > dfsofgraph(int V,vector<int> adj[]){
        vector<int > storedfs;
        vector<int > vis(V+1, 0);
        for(int i=1;i<=V;i++){
            if(!vis[i]){
                dfs(i,adj,vis,storedfs);
            }
        }
        return storedfs;
     }

};



void addedge(vector<int> adj[],int u,int v){
    adj[u].push_back(v);
    adj[v].push_back(u);
}


int main(){


    vector<int > adj[6];
    addedge(adj,1,2);
    addedge(adj,1,3);
    addedge(adj,1,4);
    addedge(adj,1,5);
    addedge(adj,2,4);
    addedge(adj,2,1);
    addedge(adj,3,1);
    addedge(adj,4,1);
    addedge(adj,4,2);

    solution obj;
    vector<int> ans = obj.dfsofgraph(5,adj);
    for(int i=0;i<ans.size();i++){
        cout<<ans[i]<<" ";
    }
    cout<<endl;
    return 0;
}


MINIMAX
#include <iostream>
using namespace std;

int minmax(bool maximizer, int finalStates[], int l, int r)
{
    if (l == r)
    {
        return finalStates[l];
    }

    int mid = (l + r) / 2;

    if (Maximizer)
    {
        return max(minmax(false, finalStates, l, mid), minmax(false, finalStates, mid + 1, r));
    }

    return min(minmax(true, finalStates, l, mid), minmax(true, finalStates, mid + 1, r));
}

int main()
{
    int n;
    cout << "Enter the number of final states N, where ( N = 2^x ) :";
    cin >> n;
    cout << "Enter the final states : \n";
    int finalStates[n];
    for (int i = 0; i < n; i++)
    {
        cin >> finalStates[i];
    }

    int ans = minmax(true, finalStates, 0, n - 1);

    cout << " Best The Maximizer can get : " << ans << endl;

    return 0;
}



Bayesian Network

#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

class Node {
public:
    vector<double> values;
};

class VariableNode : public Node {
public:
    int numValues;
};

class CPT {
public:
    vector<double> probabilities;
};

class BayesianNetwork {
public:
    VariableNode A, B;
    CPT cptA, cptB;


    double query(int a, int b) {
        double result = 0.0;
        for (int i = 0; i < A.numValues; i++) {
            for (int j = 0; j < B.numValues; j++) {
                result += cptA.probabilities[i] * cptB.probabilities[j] * (a == i) * (b == j);
            }
        }
        return result;
    }
};

int main() {
    BayesianNetwork net;
    net.A.numValues = 2;
    net.B.numValues = 2;
    net.A.values = {0.3, 0.7};
    net.B.values = {0.6, 0.4};
    net.cptA.probabilities = {0.7, 0.3};
    net.cptB.probabilities = {0.4, 0.6};

    double q = net.query(0, 1);
    cout << "P(A=0, B=1) = " << q << endl;
    return 0;
}



A* Search

from collections import deque

class Graph:
    # example of adjacency list (or rather map)
    # adjacency_list = {
    # 'A': [('B', 1), ('C', 3), ('D', 7)],
    # 'B': [('D', 5)],
    # 'C': [('D', 12)]
    # }

    def __init__(self, adjacency_list):
        self.adjacency_list = adjacency_list

    def get_neighbors(self, v):
        return self.adjacency_list[v]

    # heuristic function with equal values for all nodes
    def h(self, n):
        H = {
            'A': 1,
            'B': 1,
            'C': 1,
            'D': 1
        }

        return H[n]

    def a_star_algorithm(self, start_node, stop_node):
        # open_list is a list of nodes which have been visited, but who's neighbors
        # haven't all been inspected, starts off with the start node
        # closed_list is a list of nodes which have been visited
        # and who's neighbors have been inspected
        open_list = set([start_node])
        closed_list = set([])

        # g contains current distances from start_node to all other nodes
        # the default value (if it's not found in the map) is +infinity
        g = {}

        g[start_node] = 0

        # parents contains an adjacency map of all nodes
        parents = {}
        parents[start_node] = start_node

        while len(open_list) > 0:
            n = None

            # find a node with the lowest value of f() - evaluation function
            for v in open_list:
                if n == None or g[v] + self.h(v) < g[n] + self.h(n):
                    n = v;

            if n == None:
                print('Path does not exist!')
                return None

            # if the current node is the stop_node
            # then we begin reconstructin the path from it to the start_node
            if n == stop_node:
                reconst_path = []

                while parents[n] != n:
                    reconst_path.append(n)
                    n = parents[n]

                reconst_path.append(start_node)

                reconst_path.reverse()

                print('Path found: {}'.format(reconst_path))
                return reconst_path

            # for all neighbors of the current node do
            for (m, weight) in self.get_neighbors(n):
                # if the current node isn't in both open_list and closed_list
                # add it to open_list and note n as it's parent
                if m not in open_list and m not in closed_list:
                    open_list.add(m)
                    parents[m] = n
                    g[m] = g[n] + weight

                # otherwise, check if it's quicker to first visit n, then m
                # and if it is, update parent data and g data
                # and if the node was in the closed_list, move it to open_list
                else:
                    if g[m] > g[n] + weight:
                        g[m] = g[n] + weight
                        parents[m] = n

                        if m in closed_list:
                            closed_list.remove(m)
                            open_list.add(m)

            # remove n from the open_list, and add it to closed_list
            # because all of his neighbors were inspected
            open_list.remove(n)
            closed_list.add(n)

        print('Path does not exist!')
        return None
        
adjacency_list = {
    'A': [('B', 1), ('C', 3), ('D', 7)],
    'B': [('D', 5)],
    'C': [('D', 12)]
}
graph1 = Graph(adjacency_list)
graph1.a_star_algorithm('A', 'D')
  
  
  
Output:
Path found: ['A', 'B', 'D']
['A', 'B', 'D']

Thus, the optimal path from A to D, found using A*, is A->B->D.
