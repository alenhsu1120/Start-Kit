#include <MAPFPlanner.h>
#include <random>
#include<filesystem>
#include <thread>
#include<chrono>
#include<future>
#include<queue>
#include<unordered_map>
#include<unordered_set>
#include <cmath>
#include <string>
#include <set>
TreeNode::TreeNode() = default;
TreeNode::~TreeNode() = default;


struct AstarNode
{
    int location;
    int direction;
    int f,g,h;
    int focal;
    AstarNode* parent;
    int t = 0;
    heap_handle_t handle;
    bool closed = false;
    AstarNode(int _location,int _direction, int _g, int _h, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),parent(_parent) {}
    AstarNode(int _location,int _direction, int _g, int _h, int _t, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),t(_t),parent(_parent) {}
    bool operator<(const AstarNode&other) const{
        if(f!=other.f) return f>other.f;
        else return g<other.g;
    }
};



struct cmp
{
    bool operator()(AstarNode* a, AstarNode* b)
    {
        if(a->f == b->f) return a->g <= b->g;
        else return a->f > b->f;
    }
};




void TreeNode::add_constraint(Constraint_my &constraint){
    this->constraints.emplace_back(constraint);
}
bool TreeNode::update_solution(){
    std::vector<vector<pair<int,int>>>s;
    for(int i=0;i<node_env.num_of_agents;i++){
        std::vector<pair<int,int>>path;
        path=single_agent_plan(node_env.curr_states[i].location,
                                node_env.curr_states[i].orientation,
                                node_env.goal_locations[i].front().first,
                                i);
        

        s.emplace_back(path);
        /*
        for(auto p:path){
            cout<<"agentid "<<i<<" "<<p.first<<" "<<p.second<<endl;
        }
        */
        
    }
    this->solution=s;
    return true;
}

void TreeNode::update_cost(){
    int cost_t=0;
    for(auto &route:solution){
        
        cost_t+=route.size();
    }
    
    this->cost=cost_t;
    
   //cout<<"cost "<<this->cost<<endl;
}
/*
TreeNode MAPFPlanner::find_best_node(){
    int mi=getMinCost();
    //TreeNode ret(*env);
    for(int i=0;i<tree.size();i++){
        if(tree[i].cost==mi){
            //mi=tree[i].cost;
            //ret=tree[i];
            //cout<<"path : "<<i<<"tree[i].cost : "<<tree[i].cost<<endl;
            return tree[i];
        }
        //cout<<"i : "<<i<<endl;
    }
    return TreeNode();
}
*/
/*
int MAPFPlanner::getMinCost(){
    int min=INT_MAX;
    for(auto &node:tree){
        if(node.cost<min){
            min=node.cost;
        }
    }
    return min;
}
*/
bool MAPFPlanner::hasconflict(TreeNode&node){
    auto solutions=node.solution;
    //cout<<"hasconflict   solutions[0][0].first  "<<solutions[0][0].first<<endl;;
    for(int i=0;i<solutions.size();i++){
        for(int j=i+1;j<solutions.size();j++){
            if(hasconflict(solutions[i],solutions[j])){
                //cout<<"cnoflict i : "<<i<<" j : "<<j<<endl;
                return true;
            }
        }
    }
    return false;
}
bool MAPFPlanner::hasconflict(vector<pair<int,int>>&a,vector<pair<int,int>>&b){
    int min_idx=std::min(a.size(),b.size());
    for(int i=0;i<min_idx;i++){
        if(a[i].first==b[i].first){
            //cout<<"a[i].first"<<a[i].first<<endl;
            return true;
        }
    }
    return false;
}
bool MAPFPlanner::hasEdgeConflict(TreeNode&node){
    auto solutions=node.solution;
    for(int i=0;i<solutions.size();i++){
        for(int j=i+1;j<solutions.size();j++){
            if(hasEdgeConflict(solutions[i],solutions[j])){
                //cout<<"EdgeConflict i : "<<i<<" j : "<<j<<endl;
                return true;
            }
        }
    }
    return false;
}
bool MAPFPlanner::hasEdgeConflict(vector<pair<int,int>>&a,vector<pair<int,int>>&b){
    int min_idx=std::min(a.size(),b.size())-1;
    for(int i=0;i<min_idx;i++){
        if(a[i].first==b[i+1].first&&a[i+1].first==b[i].first){
            //cout<<"i : "<<i<<" j : "<<j<<endl;
            //cout<<"a[i].first : "<<a[i].first<<" a[i+1].first : "<<a[i+1].first<<endl;
            return true;
        }
    }
    return false;
}
Conflict_my MAPFPlanner::getFirstConflict(TreeNode&node){
    //cout<<"curr_timestep : "<<node.node_env.curr_timestep<<endl;
    auto solutions=node.solution;
    //cout<<solutions[0][0].first<<endl;
    for(int i=0;i<solutions.size();i++){
        for(int j=i+1;j<solutions.size();j++){
            int min_idx=std::min(solutions[i].size(),solutions[j].size());
            for(int k=0;k<min_idx;k++){
                if(solutions[i][k].first==solutions[j][k].first){
                    //cout<<"i :"<<i<<" j "<<j<<" solutions[i][k].first "<<solutions[i][k].first<<" k "<<k<<endl;
                    return Conflict_my(i,j,solutions[i][k].first,solutions[j][k].first,k);
                }
                    
            }
        }
    }
    for(int i=0;i<solutions.size();i++){
        for(int j=i+1;j<solutions.size();j++){
            int min_idx=std::min(solutions[i].size(),solutions[j].size())-1;
            for(int k=0;k<min_idx;k++){
                //cout<<solutions[i][k].first<<" "<<solutions[j][k+1].first<<" "<<solutions[i][k+1].first<<" "<<solutions[j][k].first<<endl;
                if(solutions[i][k].first==solutions[j][k+1].first&&solutions[i][k+1].first==solutions[j][k].first){
                    
                    //cout<<"i "<<i<<" j "<<j<<" solutions[i][k+1].first "<<solutions[i][k+1].first<<" solutions[j][k+1] "<<solutions[j][k+1].first<<" time: "<<k<<endl;
                    return Conflict_my(i,j,solutions[i][k+1].first,solutions[j][k+1].first,k);
                }
                    
            }
        }
    }
    return Conflict_my(0,0,0,0,0);
}
int MAPFPlanner::focal_heuristic(vector<vector<pair<int,int>>>&solutions){
    int numconflict=0;
    int max_t=0;
    for(auto &sol:solutions){
        //if(sol.empty()) continue;
        max_t=std::max<int>(max_t,sol.size());
    }
    
    for(int t=0;t<max_t;t++){
        //vertex conflict
        for(int i=0;i<solutions.size();i++){
            if(solutions[i].size()==0) continue;
            if(t>solutions[i].size()) continue;
            for(int j=i+1;j<solutions.size();j++){
                if(solutions[j].size()==0) continue;
                if(t>solutions[j].size()) continue;
                if(solutions[i][t].first==solutions[j][t].first){
                    numconflict++;
                    //cout<<"i "<<i<<" j "<<j<<" location "<<solutions[i][t].first<<endl;
                }
            }
        }
        
        //edge conflict
        for(int i=0;i<solutions.size();i++){
            if(solutions[i].size()==0) continue;
            if(t>solutions[i].size()-1) continue;
            for(int j=i+1;j<solutions.size();j++){
                if(solutions[j].size()==0) continue;
                if(t>solutions[j].size()-1) continue;
                if(solutions[i][t].first==solutions[j][t+1].first&&solutions[i][t+1].first==solutions[j][t].first){
                    //cout<<"i "<<i<<" j "<<j<<" location "<<solutions[i][t].first<<" "<<solutions[i][t+1].first<<endl;
                    numconflict++;
                }
            }
        }
        
    }
    return numconflict;
}

void MAPFPlanner::initialize(int preprocess_time_limit)
{
    int total_n=env->map.size();
    //cout<<env->map[65091]<<" "<<env->map[65092]<<" "<<env->map[65090]<<" "<<env->map[66151]<<endl;
    std::set<int>s;
    for(int i=0;i<total_n;i++){
        if(env->map[i]==0) s.insert(i);
    }
    srand(time(NULL));
    vector<int>dis={1,-1,env->cols,-env->cols};
    std::set<int>s2;
    int group_id=0;
    while(!s.empty()){
        auto it=s.begin();
        auto r=rand()%s.size();
        std::advance(it,r);
        int select_n=*it;
        int count=0;
        for(int i=0;i<dis.size();i++){
            if(dis[i]+select_n>=total_n||dis[i]+select_n<0) continue;
            if(env->map[dis[i]+select_n]==1) count++;
        }
        if(count>=2) special.push_back(group_id);
        int cur_x=select_n/env->cols;
        int cur_y=select_n%env->cols;
        //cout<<"current x "<<cur_x<<" "<<cur_y<<endl;
        int dis1=20/2;
        center[group_id]=select_n;
        for(int x=max(0,cur_x-dis1);x<=min(env->rows-1,cur_x+dis1);x++){
            for(int y=max(0,cur_y-dis1);y<=min(env->cols-1,cur_y+dis1);y++){
                int new_location=x*env->cols+y;
                if(env->map[new_location]==1) continue;
                int dist=sqrt(pow(x-cur_x,2)+pow(y-cur_y,2));
                if(dist<=dis1){
                    if(s.find(new_location)==s.end()) continue;
                    group[new_location]=group_id;
                    s.erase(new_location);
                }
            }
        }
        group_id++;
    }
    
    vector<pair<int,int>>d={{1,0},{-1,0},{0,1},{0,-1}};
    for(int i=0;i<env->rows;i++){
        for(int j=0;j<env->cols;j++){
            int location=i*env->cols+j;
            
            if(env->map[location]==1) continue;
            for(int k=0;k<d.size();k++){
                int new_l=(i+d[k].first)*env->cols+(j+d[k].second);
                if(new_l<0||new_l>=total_n) continue;
                if(env->map[new_l]==1) continue;
                if(group[location]==group[new_l])continue;
                int c_x=center[group[new_l]]/env->cols;
                int c_y=center[group[new_l]]%env->cols;
                int c1_x=center[group[location]]/env->cols;
                int c1_y=center[group[location]]%env->cols;
                int dist=sqrt(pow(c_x-c1_x,2)+pow(c_y-c1_y,2));
                if(std::find(special.begin(),special.end(),group[new_l])!=special.end()){
                    dist*=10;
                }
                if(std::find(special.begin(),special.end(),group[location])!=special.end()){
                    dist*=10;
                }
                graph[group[location]].insert({group[new_l],dist});
                graph[group[new_l]].insert({group[location],dist});
            }
        }
    }
    //cout<<"ok\n";
    graph2.resize(graph.size());
    for(auto g:graph){
        //cout<<"g.first "<<g.first<<endl;
        for(auto a:g.second){
            graph2[g.first].push_back({a.first,a.second});
        }
    }
    /*
    all_pair.resize(center.size());
    for(int i=0;i<center.size();i++){
        all_pair[i].resize(center.size());
    }
    for(int i=0;i<center.size();i++){
        priority_queue<pair<int,int>>q;
        vector<int>cost(center.size(),INT_MAX);
        vector<int>parent(center.size(),-1);
        q.push({0,i});
        cost[i]=0;
        while(!q.empty()){
            int d=q.top().first;
            int id=q.top().second;
            q.pop();
            for(auto a:graph[id]){
                int neighbor=a.first;
                int weight=a.second;
                if(cost[neighbor]>cost[id]+weight){
                    cost[neighbor]=cost[id]+weight;
                    parent[neighbor]=id;
                    q.push({cost[neighbor],neighbor});
                }
            }
        }
        for(int j=0;j<center.size();j++){
            int p=parent[j];
            vector<int>path;
            path.push_back(j);
            while(p!=-1){
                path.insert(path.begin(),p);
                p=parent[p];
            }
            all_pair[i][j]=path;
        }
    }
    */
    
    cout << "planner initialize done" << endl;

}

void MAPFPlanner::get_mid_target(int i){
    if(big_map==0){
        mid_target[i].push_back(env->goal_locations[i].front().first);
        return ;
    }
    int t=env->goal_locations[i].front().first;
    int t_center=group[t];
    //cout<<"t_center "<<t_center<<endl;
    vector<int>path;
    int cur=env->curr_states[i].location;
    int cur_center=group[cur];
    //cout<<"cur_center "<<cur_center<<"i "<<i<<endl;
    std::priority_queue<pair<int,int>,vector<pair<int,int>>,std::greater<pair<int,int>>>q;
    vector<long>dist(center.size(),LONG_MAX);
    vector<int>parent(center.size(),-1);
    dist[cur_center]=0;
    q.push({0,cur_center});
    while(!q.empty()){
        int id=q.top().second;
        int cur_dist=q.top().first;
        q.pop();
        for(auto n:graph2[id]){
            int v=n.first;
            long w=n.second;
            
            if(dist[v]>dist[id]+w){
                
                    //cout<<dist[v]<<" "<<dist[id]<<" "<<w<<" "<<v<<" "<<id<<endl;
                
                if(dist[id]+w>INT_MAX) dist[v]=INT_MAX;
                //cout<<dist[v]<<" "<<dist[id]<<" "<<w<<" "<<v<<" "<<id<<endl;
                else dist[v]=dist[id]+w;
                
                parent[v]=id;
                q.push({dist[v],v});
            }
        }
    }
    //cout<<"center dis "<<dist[t_center]<<endl;
    //cout<<"q ok\n";
    int p=parent[t_center];
    if(t!=center[group[t]]){
        path.push_back(t_center);
    }
    while(p!=-1){
        //cout<<"p "<<p<<endl;
        path.insert(path.begin(),p);
        p=parent[p];
    }
    //if(std::find(path.begin(),path.end(),cur_center)==path.end()){
        path.insert(path.begin(),cur_center);
    
    for(int k=0;k<path.size()-1;k++){
        for(int j=0;j<graph2[path[k]].size();j++){
            if(graph2[path[k]][j].first==path[k+1]){
                if(std::find(special.begin(),special.end(),path[k+1])!=special.end()){
                    graph2[path[k]][j].second*=10;
                    if(graph2[path[k]][j].second>INT_MAX) graph2[path[k]][j].second=INT_MAX;
                    //if(graph2[path[k]][j].second<0) cout<<"graph2 "<<graph2[path[k]][j].first<<" "<<path[k]<<endl;
                }else{
                    graph2[path[k]][j].second+=10;
                }
            }
        }
        for(int j=0;j<graph2[path[k+1]].size();j++){
            if(graph2[path[k+1]][j].first==path[k]){
                if(std::find(special.begin(),special.end(),path[k])!=special.end()){
                    graph2[path[k+1]][j].second*=10;
                    if(graph2[path[k+1]][j].second>INT_MAX) graph2[path[k+1]][j].second=INT_MAX;
                    //if(graph2[path[k+1]][j].second<0) cout<<"graph2 "<<graph2[path[k+1]][j].first<<" "<<path[k+1]<<endl;
                }else
                    graph2[path[k+1]][j].second+=10;
            }
        }
    }
    for(int j=0;j<path.size();j++){
        mid_target[i].push_back(center[path[j]]);
    }
    if(std::find(mid_target[i].begin(),mid_target[i].end(),t)==mid_target[i].end()){
        mid_target[i].push_back(t);
    }
   
    
    /*
        if(big_map==0){
            mid_target[i].push_back(env->goal_locations[i].front().first);
            return ;
        }
        int t=env->goal_locations[i].front().first;
        int t_x=t/env->cols;
        int t_y=t%env->cols;
        int cur=env->curr_states[i].location;
        int cur_x=cur/env->cols;
        int cur_y=cur%env->cols;
        int dis=sqrt(pow(t_x-cur_x,2)+pow(t_y-cur_y,2));
        //cout<<"target "<<t_x<<" "<<t_y<<" "<<cur_x<<" "<<cur_y<<" "<<dis<<" "<<i<<endl;
        int dis2=dis;
        std::pair<float,float>vec={(t_x-cur_x)*1.0/dis2,(t_y-cur_y)*1.0/dis2};
        for(int j=0;j<dis2/30;j++){
            int new_x=vec.first*30*(j+1)+cur_x;
            int new_y=vec.second*30*(j+1)+cur_y;
            int new_target=new_x*env->cols+new_y;
            //if(i==186) cout<<"nex_x "<<new_x<<" "<<new_y<<" "<<new_target<<endl;
            int move_step=1;
            while(env->map[new_target]==1){
                int n_x=new_target/env->cols;
                int n_y=new_target%env->cols;
                if(n_x<env->rows&&n_y+move_step<env->cols&&env->map[new_target+move_step]==0){
                    new_target+=move_step;
                }else if(n_x<env->rows&&n_y-move_step>=0&&env->map[new_target-move_step]==0){
                    new_target-=move_step;
                }else if(n_x+move_step<env->rows&&n_y<env->cols&&env->map[new_target+env->cols*move_step]==0){
                    new_target+=env->cols*move_step;
                }else if(n_x-move_step>=0&&n_y<env->cols&&env->map[new_target-env->cols*move_step]==0){
                    new_target-=env->cols*move_step;
                }else move_step++;
                
            }
            if(std::find(mid_target[i].begin(),mid_target[i].end(),new_target)==mid_target[i].end())
                mid_target[i].push_back(new_target);
        }
        if(std::find(mid_target[i].begin(),mid_target[i].end(),t)==mid_target[i].end())
            mid_target[i].push_back(t);
    */
        
}
void MAPFPlanner::mid_target_warehouse(int i){
    if(big_map==0){
            mid_target[i].push_back(env->goal_locations[i].front().first);
            return ;
        }
        int t=env->goal_locations[i].front().first;
        int t_x=t/env->cols;
        int t_y=t%env->cols;
        int cur=env->curr_states[i].location;
        int cur_x=cur/env->cols;
        int cur_y=cur%env->cols;
        int dis=sqrt(pow(t_x-cur_x,2)+pow(t_y-cur_y,2));
        //cout<<"target "<<t_x<<" "<<t_y<<" "<<cur_x<<" "<<cur_y<<" "<<dis<<" "<<i<<endl;
        int dis2=dis;
        std::pair<float,float>vec={(t_x-cur_x)*1.0/dis2,(t_y-cur_y)*1.0/dis2};
        for(int j=0;j<dis2/30;j++){
            int new_x=vec.first*30*(j+1)+cur_x;
            int new_y=vec.second*30*(j+1)+cur_y;
            int new_target=new_x*env->cols+new_y;
            //if(i==186) cout<<"nex_x "<<new_x<<" "<<new_y<<" "<<new_target<<endl;
            int move_step=1;
            while(env->map[new_target]==1){
                int n_x=new_target/env->cols;
                int n_y=new_target%env->cols;
                if(n_x<env->rows&&n_y+move_step<env->cols&&env->map[new_target+move_step]==0){
                    new_target+=move_step;
                }else if(n_x<env->rows&&n_y-move_step>=0&&env->map[new_target-move_step]==0){
                    new_target-=move_step;
                }else if(n_x+move_step<env->rows&&n_y<env->cols&&env->map[new_target+env->cols*move_step]==0){
                    new_target+=env->cols*move_step;
                }else if(n_x-move_step>=0&&n_y<env->cols&&env->map[new_target-env->cols*move_step]==0){
                    new_target-=env->cols*move_step;
                }else move_step++;
                
            }
            if(std::find(mid_target[i].begin(),mid_target[i].end(),new_target)==mid_target[i].end())
                mid_target[i].push_back(new_target);
        }
        if(std::find(mid_target[i].begin(),mid_target[i].end(),t)==mid_target[i].end())
            mid_target[i].push_back(t);
}
vector<Action> MAPFPlanner::thread_plan(){
    
    //std::cout<<"time "<<env->curr_timestep<<std::endl;
    auto actions = std::vector<Action>(env->curr_states.size(), Action::W);
    if(env->map_name=="sortation_large.map"||env->map_name=="warehouse_large.map"||env->map_name=="brc202d.map"||env->map_name=="Paris_1_256.map") big_map=1;
    /*
    if(!big_map){
        int change=0;
        for(int i=0;i<env->num_of_agents;i++){
            if(ret.empty()){
                change=1;
                break;
            }
            if(ret[i].back().first!=env->goal_locations[i].front().first){
                change=1;
                ret.clear();
                break;
            }
        }
        if(start==true&&change==1){
            if(high_open_.empty()){
                TreeNode start_node(*env);
            
                start_node.cost=0;

                start_node.update_solution();
                start_node.update_cost();
                start_node.focal=focal_heuristic(start_node.solution);
                auto handle=high_open_.push(start_node);
                (*handle).handle=handle;
                high_focal_.push(handle);
            }
            //cout<<"r.cost : "<<r.cost<<endl;
            finish=1;
        }
    }
    */
    if(env->curr_timestep==0){
        mid_target.resize(env->num_of_agents);
        finish_path.resize(env->num_of_agents);
        for(int i=idx;i<env->num_of_agents;i++){
            if(lock1==false){
                idx=i;
                break;
            }
            //cout<<i<<"============\n";
            if(env->map_name=="brc202d.map"||env->map_name=="Paris_1_256.map"){
                get_mid_target(i);
            }else
                mid_target_warehouse(i);
            if(i==env->num_of_agents-1) idx=0;
            //cout<<i<<" "<<mid_target[i][0]<<endl;
        }
    }else{
         if(finish==1&&conflict_finish==1){
            int flag=0;
            auto open_top1=high_focal_.top();
            TreeNode open_top=*open_top1;
            
            for(int i=idx;i<env->num_of_agents;i++){
                //if(env->curr_timestep>=260) cout<<"in for "<<i<<" ";
                if(lock1==false){
                    
                    idx=i;
                    //cout<<"lock1 false "<<idx<<endl;
                    break;
                }
                if(mid_target[i][0]==env->curr_states[i].location){
                    
                    flag=1;
                    if(env->map_name=="brc202d.map"||env->map_name=="Paris_1_256.map"){
                    finish_path[i].push_back(*mid_target[i].begin());
                    if(finish_path[i].size()>=2){
                        int i1=finish_path[i][0];
                        int i2=finish_path[i][1];
                        if(center[group[i1]]==i1&&center[group[i2]]==i2){
                            for(auto g:graph2[group[i1]]){
                                if(g.first==group[i2]){
                                    if(std::find(special.begin(),special.end(),group[i2])!=special.end()){
                                        g.second/=10;
                                    }else{
                                        g.second-=10;
                                    }
                                    
                                }
                            }
                            for(auto g:graph2[group[i2]]){
                                if(g.first==group[i1]){
                                    if(std::find(special.begin(),special.end(),group[i1])!=special.end()){
                                        g.second/=10;
                                    }else
                                        g.second-=10;
                                }
                            }
                        }
                        finish_path[i].erase(finish_path[i].begin());
                    }
                    }
                    mid_target[i].erase(mid_target[i].begin());
                    if(mid_target[i].empty()){
                        if(env->map_name=="brc202d.map"||env->map_name=="Paris_1_256.map")
                            get_mid_target(i);
                        else mid_target_warehouse(i);
                    }
                    if(env->curr_states[i].location==mid_target[i][0]){
                        mid_target[i].erase(mid_target[i].begin());
                    }
                    auto path=open_top.single_agent_plan(env->curr_states[i].location,env->curr_states[i].orientation,mid_target[i][0],i);
                    //cout<<"as task single\n";
                    //if(env->curr_timestep>=260) cout<<"single plan after "<<i<<endl;
                    if(path.empty()){
                        open_top.path_empty.push_back(i);
                        int temp=env->curr_states[i].orientation+1;
                        if(temp==4) temp=0;
                        path.push_back({env->curr_states[i].location,env->curr_states[i].orientation});
                        path.push_back({env->curr_states[i].location,temp});
                    }
                    
                    open_top.solution[i].resize(path.size(),{0,0});
                    open_top.solution[i]=path;
                    
                }
                if(i==env->num_of_agents-1) idx=0;
            }
            if(!open_top.path_empty.empty()){
                std::vector<int>no_solution;
                for(int i=0;i<open_top.path_empty.size();i++){
                    //cout<<"finish path empty vector "<<open_top.path_empty[i]<<endl;
                    vector<pair<int,int>>path;
                    //cout<<"start "<<env->curr_states[open_top.path_empty[i]].location<<" direct "<<env->curr_states[open_top.path_empty[i]].orientation<<" target "<<mid_target[open_top.path_empty[i]][0]<<endl;
                    path=open_top.single_agent_plan(env->curr_states[open_top.path_empty[i]].location,env->curr_states[open_top.path_empty[i]].orientation,mid_target[open_top.path_empty[i]][0],open_top.path_empty[i]);
                    
                    //auto path=p.single_agent_plan(env->curr_states[p.path_empty[i]].location,env->curr_states[p.path_empty[i]].orientation,mid_target[p.path_empty[i]][0],p.path_empty[i]);
                    if(path.size()==0){
                        
                        no_solution.push_back(open_top.path_empty[i]);
                        //p.path_empty.push_back(idx_1);
                        int temp=env->curr_states[open_top.path_empty[i]].orientation+1;
                        if(temp==4) temp=0;
                        path.push_back({env->curr_states[open_top.path_empty[i]].location,env->curr_states[open_top.path_empty[i]].orientation});
                        path.push_back({env->curr_states[open_top.path_empty[i]].location,temp});
                        
                    }

                    open_top.solution[open_top.path_empty[i]].resize(path.size(),{0,0});
                    open_top.solution[open_top.path_empty[i]]=path;
                }
                open_top.path_empty=no_solution;
            }
            //if(env->curr_timestep>=326050)cout<<" end \n";
            open_top.update_cost();
            open_top.focal=focal_heuristic(open_top.solution);
            high_open_.erase(open_top1);
            high_focal_.pop();
            auto handle=high_open_.push(open_top);
            (*handle).handle=handle;
            high_focal_.push(handle);
            
            if(flag==1) change=1;
            else change=0;
            //if(env->curr_timestep>=350) cout<<"for after\n";
            //cout<<"finish =1 change "<<change<<endl;
        }
    }
    if(finish==0){
        if(lock1==false) return actions;
        //cout<<"finish 0\n";
        if(high_open_.empty()){
            TreeNode start_node(*env);
            start_node.deadlock.resize(env->num_of_agents,-1);
            auto handle=high_open_.push(start_node);
            (*handle).handle=handle;
            high_focal_.push(handle);
            //cout<<"push after\n";
        }
        
        int agent=idx;
        //cout<<"agent out"<<agent<<endl;
        //TreeNode start_node(*env);
        auto top_node=high_open_.top();
        //cout<<"top_node .solution "<<top_node.solution.size()<<endl;
        //if(lock1==false) cout<<"lock1\n";
        
        vector<int>no_solution;
        while(lock1==true&&agent<env->num_of_agents){
            //cout<<"agent "<<agent<<endl;
            vector<pair<int,int>>path;
            //cout<<"mid_target "<<group[mid_target[agent][0]]<<endl;
            path=top_node.single_agent_plan(env->curr_states[agent].location,env->curr_states[agent].orientation,mid_target[agent][0],agent);
            if(path.size()==0){
                        
                top_node.path_empty.push_back(agent);
                //p.path_empty.push_back(idx_1);
                int temp=env->curr_states[agent].orientation+1;
                if(temp==4) temp=0;
                path.push_back({env->curr_states[agent].location,env->curr_states[agent].orientation});
                path.push_back({env->curr_states[agent].location,temp});
            
            }
            top_node.solution.push_back(path);
            agent++;
            if(agent==env->num_of_agents){
                finish=1;
                
            }
        }
        
       
        if(!high_open_.empty()){
            high_open_.pop();
            high_focal_.pop();
        }
        
        idx=agent;
        if(agent==env->num_of_agents) idx=0;
        //cout<<"solution size "<<top_node.solution.size()<<endl;
        top_node.update_cost();
        
        top_node.focal=focal_heuristic(top_node.solution);
        auto ha=high_open_.push(top_node);
        (*ha).handle=ha;
        high_focal_.push(ha);
        //cout<<"top_node .size()"<<top_node.solution.size()<<endl;
        //cout<<"high_open_.solution "<<high_open_.top().solution.size()<<endl;
        //auto h=high_focal_.top();
        //TreeNode temp=*h;
        //cout<<"high_focal_.solution "<<temp.solution.size()<<endl;
    }
    
    if(ret.empty()&&finish==1) change=1;
        int best_cost=high_open_.top().cost;
        while(lock1==true&&finish==1&&!high_open_.empty()){

            //cout<<"open : "<<high_open_.size()<<" focal : "<<high_focal_.size()<<" focal "<<(*high_focal_.top()).focal<<endl;
            int old_cost=best_cost;
            best_cost=high_open_.top().cost;
            if(best_cost>old_cost){
                auto it=high_open_.ordered_begin();
                auto it_end=high_open_.ordered_end();
                for(;it!=it_end;it++){
                    int val=it->cost;
                    const auto &a=*it;
                    if(std::find(high_focal_.begin(),high_focal_.end(),a.handle)!=high_focal_.end()) continue;
                    if(val<=best_cost*1.3){
                        const TreeNode&n=*it;
                        high_focal_.push(n.handle);
                    }   
                    if(val>best_cost*1.3) break;
                }
            }
            auto h=high_focal_.top();
            TreeNode p=*h;
            p.node_env=(*env);
            high_focal_.pop();
            high_open_.erase(h);
            
            if(!p.path_empty.empty()){
                std::vector<int>no_solution;
                for(int i=0;i<p.path_empty.size();i++){
                    //cout<<"path empty vector "<<p.path_empty[i]<<endl;
                    vector<pair<int,int>>path;
                    
                    path=p.single_agent_plan(env->curr_states[p.path_empty[i]].location,env->curr_states[p.path_empty[i]].orientation,mid_target[p.path_empty[i]][0],p.path_empty[i]);
                    
                    //auto path=p.single_agent_plan(env->curr_states[p.path_empty[i]].location,env->curr_states[p.path_empty[i]].orientation,mid_target[p.path_empty[i]][0],p.path_empty[i]);
                    if(path.size()==0){
                        
                        no_solution.push_back(p.path_empty[i]);
                        //p.path_empty.push_back(idx_1);
                        int temp=env->curr_states[p.path_empty[i]].orientation+1;
                        if(temp==4) temp=0;
                        path.push_back({env->curr_states[p.path_empty[i]].location,env->curr_states[p.path_empty[i]].orientation});
                        path.push_back({env->curr_states[p.path_empty[i]].location,temp});
                        
                    }

                    p.solution[p.path_empty[i]].resize(path.size(),{0,0});
                    p.solution[p.path_empty[i]]=path;
                }
                p.path_empty=no_solution;
            }
            
            
            if(!hasconflict(p)&&!hasEdgeConflict(p)){
                //cout<<"no conflict----\n"; 
                
                high_open_.clear();
                high_focal_.clear();
                p.constraints.clear();
                p.deadlock.clear();
                p.deadlock.resize(env->num_of_agents);
                auto handle1=high_open_.push(p);
                (*handle1).handle=handle1;
                high_focal_.push(handle1);
                ret=p.solution;
                
                deadlock.clear();
                 
                break;
            }
            else if(hasconflict(p)){
                //cout<<"conflict\n";
                Conflict_my conflict =getFirstConflict(p);
                /*
                string str=std::to_string(conflict.conflict_agent.first)+" "+std::to_string(conflict.conflict_agent.second);
                deadlock[str]++;
                if(deadlock[str]>=10){
                    p.deadlock[conflict.conflict_agent.first]=conflict.location1;
                    p.deadlock[conflict.conflict_agent.second]=conflict.location2;
                }
                */
                //tree.pop_back();
                
                for(int i=0;i<2;i++){
                    TreeNode A=p;
                    int idx_1=conflict.conflict_agent.first;
                    int idx_2=conflict.conflict_agent.second;
                    //cout<<"idx_1  "<<idx_1<<" idx_2  "<<idx_2<<" location "<<conflict.location1<<" time "<<conflict.time<<endl;
                    auto new_constraint=Constraint_my(conflict.location1,conflict.conflict_agent.first,conflict.time);
                    if(i==1){
                        new_constraint=Constraint_my(conflict.location2,conflict.conflict_agent.second,conflict.time);
                    }  
                    A.add_constraint(new_constraint);
                    //cout<<"A .con back agentid"<<A.constraints.back().agentid<< "location "<<A.constraints.back().location<<" time "<<A.constraints.back().time<<endl;
                    //cout<<"before a.solution : "<<A.solution[idx_1].size()<<endl;
                    vector<pair<int,int>>path1;
                    
                    path1=A.single_agent_plan(env->curr_states[idx_1].location,env->curr_states[idx_1].orientation,mid_target[idx_1][0],idx_1);
                    
                    if(path1.size()==0){
                        /*
                        cout<<"vertex path1 empty =============\n";
                        cout<<"agentid "<<idx_1<<" "<<env->curr_states[idx_1].location<<endl;
                        cout<<"vertex single_plan "<<env->curr_states[idx_1].location<<" "<<env->curr_states[idx_1].orientation<<" target "<<mid_target[idx_1][0]<<endl;
                        for(auto a:A.constraints){
                            cout<<"agentid "<<a.agentid<<" time "<<a.time<<" location "<<a.location<<endl;
                        }
                        cout<<"-----\n";
                        */
                        if(std::find(A.path_empty.begin(),A.path_empty.end(),idx_1)==A.path_empty.end()){
                            A.path_empty.push_back(idx_1);
                        }
                        
                        int temp=env->curr_states[idx_1].orientation+1;
                        if(temp==4) temp=0;
                        path1.push_back({env->curr_states[idx_1].location,env->curr_states[idx_1].orientation});
                        path1.push_back({env->curr_states[idx_1].location,temp});
                        /*
                        for(auto a:A.constraints){
                            cout<<"location "<<a.location<<" time "<<a.time<<" agentid "<<a.agentid<<endl;
                        }
                        */
                        //cout<<"empty\n";
                        //continue;
                    }
                    A.solution[idx_1].clear();
                    A.solution[idx_1]=path1;
                    vector<pair<int,int>>path2;
                    path2=A.single_agent_plan(env->curr_states[idx_2].location,env->curr_states[idx_2].orientation,mid_target[idx_2][0],idx_2);
                    
                    if(path2.size()==0){
                        /*
                        cout<<"vertex path2 empty =============\n";
                        cout<<"agentid "<<idx_2<<" "<<env->curr_states[idx_2].location<<endl;
                        cout<<"vertex single_plan "<<env->curr_states[idx_2].location<<" "<<env->curr_states[idx_2].orientation<<" target "<<mid_target[idx_2][0]<<endl;
                        for(auto a:A.constraints){
                            cout<<"agentid "<<a.agentid<<" time "<<a.time<<" location "<<a.location<<endl;
                        }
                        cout<<"-----\n";
                        */
                        if(std::find(A.path_empty.begin(),A.path_empty.end(),idx_2)==A.path_empty.end()){
                            A.path_empty.push_back(idx_2);
                        }
                        
                        int temp=env->curr_states[idx_2].orientation+1;
                        if(temp==4) temp=0;
                        path2.push_back({env->curr_states[idx_2].location,env->curr_states[idx_2].orientation});
                        path2.push_back({env->curr_states[idx_2].location,temp});
                        /*
                        for(auto a:A.constraints){
                            cout<<"location "<<a.location<<" time "<<a.time<<" agentid "<<a.agentid<<endl;
                        }
                        */
                        //cout<<"empty\n";
                        //continue;
                    }
                    A.solution[idx_2].clear();
                    A.solution[idx_2]=path2;
                    
                    
                    A.update_cost();
                   
                    A.focal=focal_heuristic(A.solution);
                    
                    //cout<<"A.focal : "<<A.focal<<endl;
                    if(A.cost<INT_MAX){
                        A.deadlock.resize(env->num_of_agents,-1);
                        auto handle=high_open_.push(A);
                        (*handle).handle=handle;
                        if(A.cost<=best_cost*1.3){
                            high_focal_.push(handle);
                        }
                    }
                    //cout<<"push after--\n";
                }
                
            }else if(hasEdgeConflict(p)){
                //cout<<"EdgeConflict----\n";
                auto conflict=getFirstConflict(p);
                /*
                string str=std::to_string(conflict.conflict_agent.first)+" "+std::to_string(conflict.conflict_agent.second);
                deadlock[str]++;
                if(deadlock[str]>=10){
                    p.deadlock[conflict.conflict_agent.first]=conflict.location1;
                    p.deadlock[conflict.conflict_agent.second]=conflict.location2;
                }
                cout<<"deadlock\n";
                for(auto d:deadlock){
                    cout<<"first "<<d.first<<" -> "<<d.second<<endl;
                }
                */
                //cout<<conflict.location1<<" "<<conflict.location2<<" "<<conflict.time<<" "<<conflict.conflict_agent.first<<" "<<conflict.conflict_agent.second<<endl;
                for(int i=0;i<2;i++){
                    TreeNode A=p;
                    int idx_1=conflict.conflict_agent.first;
                    int idx_2=conflict.conflict_agent.second;
                    //cout<<"A.cost : "<<A.cost<<"p.cost : "<<p.cost<<endl;
                    auto new_constraint=Constraint_my(conflict.location2,conflict.conflict_agent.first,conflict.time);
                    auto constraint2=Constraint_my(conflict.location1,conflict.conflict_agent.first,conflict.time+1);
                    if(i==1){
                        new_constraint=Constraint_my(conflict.location1,conflict.conflict_agent.second,conflict.time);
                        constraint2=Constraint_my(conflict.location2,conflict.conflict_agent.second,conflict.time+1);
                    }
                    //cout<<"before: A.constraints.size():  "<<A.constraints.size()<<endl;
                    if(new_constraint.time==0){
                        A.add_constraint(constraint2);
                    }else{
                        A.add_constraint(new_constraint);
                        A.add_constraint(constraint2);
                    }
                    
                    //cout<<"A.constraints.size()  "<<A.constraints.size()<<endl;
                    //cout<<"after: A.constraints.size():  "<<A.constraints.size()<<endl;
                    /*
                    cout<<"constraint\n";
                    for(int i=0;i<A.constraints.size();i++){
                        cout<<A.constraints[i].location<<" "<<A.constraints[i].agentid<<" "<<A.constraints[i].time<<endl;
                    }
                    cout<<"constraint after\n";
                    */
                    vector<pair<int,int>>path1;
                    
                        path1=A.single_agent_plan(env->curr_states[idx_1].location,env->curr_states[idx_1].orientation,mid_target[idx_1][0],idx_1);
                        //else path1=A.single_agent_plan(env->curr_states[idx_1].location,env->curr_states[idx_1].orientation,env->goal_locations[idx_1].front().first,idx_1);
                        if(path1.size()==0){
                            /*
                            cout<<"edge path1 empty =============\n";
                            cout<<"agentid "<<idx_1<<" "<<env->curr_states[idx_1].location<<endl;
                            cout<<"edge single_plan "<<env->curr_states[idx_1].location<<" "<<env->curr_states[idx_1].orientation<<" target "<<mid_target[idx_1][0]<<endl;
                            for(auto a:A.constraints){
                                cout<<"agentid "<<a.agentid<<" time "<<a.time<<" location "<<a.location<<endl;
                            }
                            cout<<"-----\n";
                            */
                            if(std::find(A.path_empty.begin(),A.path_empty.end(),idx_1)==A.path_empty.end()){
                                A.path_empty.push_back(idx_1);
                            }
                            
                            int temp=env->curr_states[idx_1].orientation+1;
                            if(temp==4) temp=0;
                            path1.push_back({env->curr_states[idx_1].location,env->curr_states[idx_1].orientation});
                            path1.push_back({env->curr_states[idx_1].location,temp});
                            /*
                            for(auto a:A.constraints){
                                cout<<"location "<<a.location<<" time "<<a.time<<" agentid "<<a.agentid<<endl;
                            }
                            */
                            //cout<<"e_empty\n";
                            //continue;
                        }
                        //cout<<"A.idx_1 "<<A.solution[idx_1].size()<<" - ";
                        A.solution[idx_1].clear();
                        A.solution[idx_1]=path1;
                        //cout<<"A.idx_1 "<<A.solution[idx_1].size()<<endl;;
                        vector<pair<int,int>>path2;
                        path2=A.single_agent_plan(env->curr_states[idx_2].location,env->curr_states[idx_2].orientation,mid_target[idx_2][0],idx_2);
                        //else path2=A.single_agent_plan(env->curr_states[idx_2].location,env->curr_states[idx_2].orientation,env->goal_locations[idx_2].front().first,idx_2);
                        if(path2.size()==0){
                            /*
                            cout<<"edge path2 empty =============\n";
                            cout<<"agentid "<<idx_2<<" "<<env->curr_states[idx_2].location<<endl;
                            cout<<"edge single_plan "<<env->curr_states[idx_2].location<<" "<<env->curr_states[idx_2].orientation<<" target "<<mid_target[idx_2][0]<<endl;
                            for(auto a:A.constraints){
                                cout<<"agentid "<<a.agentid<<" time "<<a.time<<" location "<<a.location<<endl;
                            }
                            cout<<"-----\n";
                            */
                            if(std::find(A.path_empty.begin(),A.path_empty.end(),idx_2)==A.path_empty.end()){
                                A.path_empty.push_back(idx_2);
                            }
                            
                            int temp=env->curr_states[idx_2].orientation+1;
                            if(temp==4) temp=0;
                            path2.push_back({env->curr_states[idx_2].location,env->curr_states[idx_2].orientation});
                            path2.push_back({env->curr_states[idx_2].location,temp});
                            /*
                            for(auto a:A.constraints){
                                cout<<"location "<<a.location<<" time "<<a.time<<" agentid "<<a.agentid<<endl;
                            }
                            */
                            //cout<<"e_empty\n";
                            //continue;
                        }
                        //cout<<"A.idx_2 "<<A.solution[idx_2].size()<<" - ";
                        A.solution[idx_2].clear();
                        A.solution[idx_2]=path2;
                        //cout<<"A.idx_2 "<<A.solution[idx_2].size()<<endl;
                    
                    //cout<<"e_path--\n";
                    
                    //if(A.update_solution()==false) continue;
                    
                    
                    A.update_cost();
                    A.focal=focal_heuristic(A.solution);
                    //cout<<"A.focal : "<<A.focal<<endl;
                    if(A.cost<INT_MAX){
                        A.deadlock.resize(env->num_of_agents,-1);
                        auto handle=high_open_.push(A);
                        (*handle).handle=handle;
                        if(A.cost<=best_cost*1.3){
                            high_focal_.push(handle);
                        }
                    }
                    //cout<<"e_push--\n";
                }
                //remove_node(tree,p);
            }
            
        }
        
        if(lock1==false){
            conflict_finish=0;
            //cout<<"conflict_finish\n";
        }else{
            
            //cout<<"action\n";
            //cout<<ret.size()<<endl;
            for(int i=0;i<env->num_of_agents;i++){
                
                if(ret[i].size()==1){
                    actions[i]=Action::W;
                }
                else if(ret[i][1].first!=env->curr_states[i].location){
                    actions[i]=Action::FW;
                    
                }
                else if(ret[i][1].second!=env->curr_states[i].orientation){
                    int incur=ret[i][1].second-env->curr_states[i].orientation;
                    if(incur==1||incur==-3){
                        actions[i]=Action::CR;
                        
                    }
                    else if(incur==-1||incur==3){
                        actions[i]=Action::CCR;
                        
                    }
                }
                
                ret[i].erase(ret[i].begin());
                
            }
            //cout<<"\n";
            
            auto tmp=high_focal_.top();
            auto node=*tmp;
            
            node.solution.resize(ret.size());
            node.solution=ret;
            node.node_env=(*env);
                //if(env->curr_timestep>=80) cout<<"i "<<i<<endl;
            node.deadlock.clear();
            node.deadlock.resize(env->num_of_agents,-1);
            if(!node.path_empty.empty()) node.path_flag=1;
            else node.path_flag=0;
            //cout<<"node.solution "<<node.solution[1][0].first<<" "<<node.solution[1][0].second<<endl;
            node.update_cost();
            node.focal=focal_heuristic(node.solution);

            high_open_.pop();
            high_focal_.pop();
            auto pu=high_open_.push(node);
            (*pu).handle=pu;
            high_focal_.push(pu);
            conflict_finish=1;
        }
        
       
        return actions;
    
    //cout<<"ret[0][1].first"<<ret[0][1].first<<endl;

}

// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    lock1=true;
    actions = std::vector<Action>(env->curr_states.size(), Action::W);
    std::packaged_task<std::vector<Action>()>task1(std::bind(&MAPFPlanner::thread_plan,this));
    auto future1=task1.get_future();
    auto task_td1=std::thread(std::move(task1));
    std::future_status ret=future1.wait_for(std::chrono::milliseconds(700*time_limit));
    if(ret==std::future_status::ready){
        
        task_td1.join();
        start=true;
        //cout<<"ready\n";
        actions=future1.get();
        
    }else if(ret==std::future_status::timeout){
        lock1=false;
        start=false;
        
        task_td1.join();
        actions=future1.get();
        //cout<<"timeout\n";
        //cout<<tree.size()<<endl;
        lock1=true;
    }  
}
bool TreeNode::is_constraint(int agentid,int location,int time,std::vector<Constraint_my>&constraints){
    //cout<<"location : "<<location<<" agentid : "<<agentid<<" time : ---"<<time<<endl;
    for(Constraint_my c:constraints){
        //cout<<"c.location : "<<c.location<<" c.agentid : "<<c.agentid<<" c.time : "<<c.time<<endl;
        if(agentid==c.agentid){
            if(time==c.time&&location==c.location&&c.location!=node_env.curr_states[agentid].location){
                //cout<<"agentid: "<<agentid<<" location: "<<location <<endl;
                return true;
            }
        }
    }
    return false;
}


vector<pair<int,int>> TreeNode::single_agent_plan(int start,int start_direct,int end,int agentid)
{
    //cout<<"start "<<start<<" start_direct "<<start_direct<<" end "<<end<<" agentid "<<agentid<<" time "<<node_env.curr_timestep<<"-----------------------"<<endl;
    
    int ob=deadlock[agentid];
    
    if(ob!=-1){
        //cout<<node_env.map[58411]<<" "<<node_env.map[58413]<<" "<<node_env.map[57882]<<" "<<node_env.map[58942]<<endl;
    }
    /*
    if(ob!=-1){
        cout<<"agentid "<<agentid<<" ob "<<ob<<endl;
        cout<<"start "<<start<<" start_direct "<<start_direct<<" end "<<end<<" agentid "<<agentid<<" time "<<node_env.curr_timestep<<"-----------------------"<<endl;
    }
    */
    auto start_state=State_my(start,0,start_direct);
    std::vector<pair<int,int>> path;
    if(start==end){
        path.push_back({start,start_direct});
        path.push_back({start,start_direct});
        return path;
    }
    open_t open_set;
    focal_t focal_set;
    std::unordered_map<State_my,heap_handle_t,std::hash<State_my>>state_heap;
    std::unordered_set<State_my,std::hash<State_my>> closed_set;
    
    std::unordered_map<State_my,State_my,std::hash<State_my>>cameFrom;
    Node start_node(start_state,getManhattanDistance(start,end),0,0);
    
    auto handle=open_set.push(start_node);
    state_heap.insert(std::make_pair<>(start_state,handle));
    (*handle).handle=handle;
    //std::cout<<"*handle\n";
    focal_set.push(handle);
    int best_cost=(*handle).fscore;
    //AstarNode* s = new AstarNode(start, start_direct, 0, getManhattanDistance(start,end), nullptr);
    //s->t=0;
    //open_list.push(s);
    //all_nodes[start*4 + start_direct] = s;
    //int find=0;
    //cout<<"agentid : "<<agentid<<endl;
    int count=0;
    //int count_flag=0;
    
    while (!open_set.empty())
    {
        //cout<<"focal top "<<(*focal_set.top()).state.location<<endl;
        
        //cout<<"low open size "<<open_set.size()<<" focal size "<<focal_set.size()<<endl;
        //mx=std::max(mx,(int)open_set.size());
        
        if(!path_flag){
            //if(agentid==758) cout<<open_set.size()<<" ";
            count++;
            if(count>=5000) break;
        }
        
        int old_best=best_cost;
        best_cost=open_set.top().fscore;
        
        //cout<<"best cost "<<best_cost<<" location "<<open_set.top().state.location<<endl;
        if(best_cost>old_best){
            auto iter=open_set.ordered_begin();
            auto iter_end=open_set.ordered_end();
            for(;iter!=iter_end;iter++){
                const auto&s=*iter;
                //cout<<"fscore in open : "<<s.fscore<<endl;
                //if(std::find(focal_set.begin(),focal_set.end(),s.handle)!=focal_set.end()) continue;
                int val=(*iter).fscore;
                if(val>old_best*1.3&&val<=best_cost*1.3){
                    const Node&n=*iter;
                    focal_set.push(n.handle);
                    //cout<<"cost "<<val<<endl;
                    //cout<<"best_cost>old_best\n";
                }
                if(val>best_cost*1.3) break;
            }
            //cout<<"------------"<<endl;
        }
        //cout<<"best\n";
        //cout<<"after open size "<<open_set.size()<<" focal size "<<focal_set.size()<<endl;
        auto curr=focal_set.top();
        Node current=*curr;
        //cout<<"current "<<current->state.location<<" "<<current->state.orientation<<endl;
        //if(current->parent!=NULL) cout<<" current.parent "<<(current->parent)->state.location<<" "<<(current->parent)->state.orientation<<endl;
        if(current.state.location==end){
            /*
            if(agentid==1){
                cout<<"------------\n";
            for(auto c:cameFrom){
                cout<<"neighbor "<<c.first.location<<"-"<<c.first.orientation<<" time "<<c.first.timestep<<" current "<<c.second.location<<"-"<<c.second.orientation<<" time "<<c.second.timestep<<endl;
            }
            }
            */
            auto it=cameFrom.find(current.state);
            
                //cout<<"agentid "<<agentid<<endl;
            //auto it2=it->second;
            while(it!=cameFrom.end()){
                path.insert(path.begin(),std::make_pair<>(it->first.location,it->first.orientation));
                
                    //cout<<"neighbor "<<it->first.location<<"-"<<it->first.orientation<<" cur "<<it->second.location<<"-"<<it->second.orientation<<" ";
                //it2=it->second;
                it=cameFrom.find(it->second);
            }
            //if(start_state.location!=it2.location) cout<<"-----------===========--------------------\n";
            path.insert(path.begin(),{start_state.location,start_state.orientation});
            break;
        }
        focal_set.pop();
        open_set.erase(curr);
        //cout<<"open_set erase--\n";
        state_heap.erase(current.state);
        closed_set.insert(current.state);
        //cout<<"open_set after\n";
        //closed_set.insert(current.state);
        //cout<<"pop\n";
        std::vector<State_my>neighbors=getNeighbors(current.state);
        for(auto neighbor:neighbors){
            //cout<<"neighbor----------\n";
            
            if(is_constraint(agentid,neighbor.location,current.state.timestep+1,constraints)){
                //cout<<"con agentid "<<agentid<<" location "<<neighbor.location<<" time "<<current.state.timestep+1<<endl;
                continue;
            }
            if(ob!=-1){
                if(neighbor.location==ob&&ob!=end&&ob!=start) continue;
                
            }
            /*
            int flag=0;
            if(neighbor.timestep==1){
                for(int j=0;j<node_env.curr_states.size();j++){
                    //cout<<"curr "<<curr.location<<endl;
                    if(neighbor.location==node_env.curr_states[j].location&&agentid!=j){
                        cout<<"neigbor location "<<neighbor.location<<" j "<<j<<endl;
                        flag=1;
                        break;
                    }
                }
            }
            if(flag==1) continue;
            */
            //cout<<"constratint\n";
            if(closed_set.find(neighbor)==closed_set.end()){
                int curr_g=current.gscore+1;
                
                auto iter=state_heap.find(neighbor);
                if(iter==state_heap.end()){
                    //cout<<"location "<<neighbor.location<<" "<<neighbor.orientation<<endl;
                    int fscore=curr_g+getManhattanDistance(neighbor.location,end);
                    int focal=current.focal+vertex_conflict(neighbor,agentid)+edge_conflict(current.state,neighbor,agentid);
                    //if(current.state.location==neighbor.location) focal+=node_env.num_of_agents;
                    //cout<<"conflict\n";
                    Node next_node(neighbor,fscore,curr_g,focal);
                    
                    //Node *next_node=new Node(neighbor,fscore,curr_g,focal,&current);
                    //cout<<"next node "<<(next_node)->state.location<<" current node "<<(next_node->parent)->state.location<<endl;
                    auto handle=open_set.push(next_node);
                    (*handle).handle=handle;
                    state_heap.insert(std::make_pair<>(neighbor,handle));
                    
                    if(fscore<=best_cost*1.3){
                        focal_set.push(handle);
                        //cout<<"handle.fscore 1 "<<(*handle).fscore<<endl;
                        //cout<<"focal push\n";
                    }
                    //state_heap.insert(std::make_pair<>(neighbor,handle));
                    //cout<<"iter==end()\n";
                }else{
                    auto handle=iter->second;
                    if(curr_g>=(*handle).gscore) continue;
                    int last_g=(*handle).gscore;
                    int last_f=(*handle).fscore;
                    int d=last_g-curr_g;
                    (*handle).gscore=curr_g;
                    (*handle).fscore-=d;
                    open_set.update(handle);
                    if(std::find(focal_set.begin(),focal_set.end(),handle)!=focal_set.end()) continue;
                    if((*handle).fscore<=best_cost*1.3){
                        //cout<<"handle.fscore 2 "<<(*handle).fscore<<endl;
                        focal_set.push(handle);
                        
                        //cout<<"focal push "<<(**handle).state.location<<endl;
                    }
                    //cout<<"iter!=end()\n";
                }
                cameFrom.erase(neighbor);
                cameFrom.insert(std::make_pair<>(neighbor,current.state));
                /*
                if(agentid==1){
                cout<<"neighbor "<<neighbor.location<<"-"<<neighbor.orientation<<" time "<<neighbor.timestep<<" current "<<current.state.location<<"-"<<current.state.orientation<<" time "<<current.state.timestep<<endl;
                if(cameFrom.find(State_my(492,4,1))!=cameFrom.end()) cout<<"find-----------\n";
                }
                */
                
            }
        }
        
        
        
    }
   //if(path.empty()) cout<<"path empty()\n";
   
    return path;
}
int TreeNode::vertex_conflict(State_my&s,int agentid){
    int numconflict=0;
    for(int i=0;i<solution.size();i++){
        if(i!=agentid&&solution[i].size()>s.timestep){
            if(s.location==solution[i][s.timestep].first){
                numconflict++;
            }
        }
    }
    return numconflict;
}

int TreeNode::edge_conflict(State_my&s1,State_my&s2,int agentid){
    int numconflict=0;
    for(int i=0;i<solution.size();i++){
        if(agentid!=i&&solution[i].size()>s2.timestep){
            if(s2.location==solution[i][s1.timestep].first&&s1.location==solution[i][s2.timestep].first){
                numconflict++;
            }
        }
    }
    return numconflict;
}

int TreeNode::getManhattanDistance(int loc1, int loc2)
{
    int loc1_x = loc1/node_env.cols;
    int loc1_y = loc1%node_env.cols;
    int loc2_x = loc2/node_env.cols;
    int loc2_y = loc2%node_env.cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}


bool TreeNode::validateMove(int loc, int loc2)
{
    int loc_x = loc/node_env.cols;
    int loc_y = loc%node_env.cols;

    if (loc_x >= node_env.rows || loc_y >= node_env.cols || node_env.map[loc] == 1)
        return false;

    int loc2_x = loc2/node_env.cols;
    int loc2_y = loc2%node_env.cols;
    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1)
        return false;
    
    return true;

}


std::vector<State_my> TreeNode::getNeighbors(State_my &current)
{
    std::vector<State_my> neighbors;
    //forward
    int candidates[4] = { current.location + 1,current.location + node_env.cols, current.location - 1, current.location - node_env.cols};
    int forward = candidates[current.orientation];
    int new_direction = current.orientation;
    //cout<<"getNeighbors  agentid: "<<agentid<<" forward : "<<forward<<" time : "<<time<<endl;
    
    if (forward>=0 && forward < node_env.map.size() && validateMove(forward,current.location))
        neighbors.emplace_back(State_my(forward,current.timestep+1,new_direction));
    //turn left
    new_direction = current.orientation-1;
    if (new_direction == -1)
        new_direction = 3;
    neighbors.emplace_back(State_my(current.location,current.timestep+1,new_direction));
    //turn right
    new_direction = current.orientation+1;
    if (new_direction == 4)
        new_direction = 0;
    neighbors.emplace_back(State_my(current.location,current.timestep+1,new_direction));
    //neighbors.emplace_back(State_my(current.location,current.timestep+1,current.orientation));//wait
    return neighbors;
}
