function run_sim(arg,origin,goal,u,dt,num_points,b,track)

switch arg
    case 'B'
        sim_BRRT(origin,goal,u,dt,num_points,b,track);
    case 'S'
        sim_Convergent_SST(origin,goal,u,dt,num_points,b,track)
    otherwise
        
end

