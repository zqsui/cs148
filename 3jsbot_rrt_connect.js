//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// CS148: 
// implement RRT-Connect by Kuffner and LaValle (2000)
//    paper link: http://msl.cs.uiuc.edu/~lavalle/papers/KufLav00.pdf

// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by robot_collision_test()

/*
CS148: reference code has functions for:

    tree_add_vertex
    tree_add_edge
    random_config
    new_config
    nearest_neighbor
    rrt_extend
    rrt_connect
    find_path
    path_dfs
*/


function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // CS148: add necessary RRT initialization here
    value_domain = new Array(); 
    value_domain[0] =  robot_boundary[1][0] - robot_boundary[0][0];
    value_domain[1] = 0;
    value_domain[2] = robot_boundary[1][2] - robot_boundary[0][2];
    value_domain[3] = 0;
    value_domain[4] = 2 * Math.PI;
    value_domain[5] = 0;   
    for ( var i = 6 ; i < q_start_config.length; i++)
        value_domain[i] = 2 * Math.PI;

    //q_start_config = random_config();
    
    console.log(q_start_config);

    Tree_a = tree_init(q_start_config, "a"); 
    Tree_b = tree_init(q_goal_config, "b");

    //q_new = new Array(q_start_config.length);


    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    console.log("planner initialized");

    robot_path = new Array();

    ta = {}
    tb = {}
    meet_vertex_a = 0;
    meet_vertex_b = 0;
}

function random_config(){
    var q_random_config = new Array(q_start_config.length);


    for( var i = 0 ; i < q_random_config.length; i++ )
        q_random_config[i] = Math.random() * value_domain[i];
    q_random_config[0] += robot_boundary[0][0];
    q_random_config[2] += robot_boundary[0][2];

    /*
    while (robot_collision_test(q_random_config)){

        for( var i = 0 ; i < q_random_config.length; i++ )
            q_random_config[i] = Math.random() * value_domain[i];
    }*/

    return q_random_config
}

function robot_rrt_planner_iterate() {

    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (generating_motion_plan && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

        // CS148: implement RRT iteration here
        var q_random_config = random_config();

        if ( rrt_extend(Tree_a, q_random_config) != "Trapped" )
        {
            if ( rrt_connect(Tree_b, Tree_a.vertices[Tree_a.newest].vertex) == "Reached" )
            {   
                generate_path();
                return "Reached";
            }
        }

        var tmp_tree = Tree_a;
        Tree_a = Tree_b;
        Tree_b = tmp_tree;


    }

    // return path not currently found
    return false;
}

function nearest_neighbor(T, q)
{
    var q_near = new Array(q_start_config.length);
    var min_dis = 100000000;
    var index = -1;
    for ( var i = 0 ; i < T.vertices.length; i++ )
    {
         dis = 0;
         for ( var j = 0 ; j < 11; j++)
            dis += (q[j] - T.vertices[i].vertex[j]) * (q[j] - T.vertices[i].vertex[j]);
         //(q[0] - T.vertices[i].vertex[0]) * (q[0] - T.vertices[i].vertex[0]) + (q[2] - T.vertices[i].vertex[2]) * (q[2] - T.vertices[i].vertex[2]);
         dis = Math.sqrt(dis)
         if ( dis < min_dis )
         {
            min_dis = dis;
            index = i;
         }


    }

    return index;
}


function new_config(q, q_near, q_new)
{
    var eps = 1.5

    var diff = new Array(q_start_config.length);

    for ( var i = 0 ; i < diff.length ; i++ )
        diff[i] = q[i] - q_near[i]


    for (var j = 1 ; j <= 10 ; j++ )
    {
        var vec_len = vector_length(diff);
        if ( vec_len < eps )
        {
            vec_len = eps
            /*for ( var i = 0 ; i < q_new.length ; i++ )
                q_new[i] = q_near[i] + eps*/

        }
        
        for ( var i = 0 ; i < q_new.length ; i++ )
          q_new[i] = q_near[i] + eps * diff[i]/vec_len * j / 10;
        

      /*  for ( var i = 0 ; i < q_new.length ; i++ )
            q_new[i] = q_near[i] + eps * diff[i]*/

        //window.cancelAnimationFrame(requestID+1);
        if ( robot_collision_test(q_new) )
            return false
    }
    
    return true

}

function rrt_extend(T, q){
    var n_index = nearest_neighbor(T, q)
    var q_near = T.vertices[n_index].vertex;
    q_new = new Array(q_start_config.length);

    if ( new_config(q, q_near, q_new) )
    {
        var index = T.vertices.length;
        T.vertices[index] = {};
        T.vertices[index].vertex = q_new;
        T.newest = index;
        T.vertices[index].p_vertex = n_index;
        add_config_origin_indicator_geom(T.vertices[index]);
        if ( v_equal(q, q_new) )
        {
            ta = T;
            meet_vertex_a = index;
            if ( ta.id == Tree_a.id )
                tb = Tree_b;
            else
                tb = Tree_a;
            for (var j = 0 ; j < tb.vertices.length; j++ )
                if ( v_equal(tb.vertices[j].vertex, q) )
                {
                    meet_vertex_b = j;
                    break;
                }

            return "Reached";
        }
        else
            return "Advanced";
    }

    return "Trapped";   
}

function rrt_connect(T, q){
    var S = ""

    do
    {
        S = rrt_extend(T, q);
        console.log(S);
    }while( S == "Advanced")
    //console.log(S)

    return S
}

function v_equal(q1, q2)
{
    for ( var i = 0 ; i < q1.length ; i++ )
        if ( Math.abs(q1[i] - q2[i]) > 0.001 )
            return false
    return true
}

function generate_path()
{
    var index = meet_vertex_a;
    while (index != 0)
    {
        robot_path.push(ta.vertices[index]);
        index = ta.vertices[index].p_vertex;
        //console.log(index);
    }
    robot_path.push(ta.vertices[index]);

    robot_path.reverse();

    index = meet_vertex_b;
    while (index != 0)
    {
        robot_path.push(tb.vertices[index]);
        index = tb.vertices[index].p_vertex;
        //console.log(index);
    }
    robot_path.push(tb.vertices[index]);

    var flag = false;
    for ( var i=0 ; i < 11; i++ )
        if ( robot_path[0].vertex[i] != 0)
        {
            flag = true;
            break;
        }           

    if ( !flag ) robot_path.reverse();

    for( var i = 0 ;i < robot_path.length; i++ )
         robot_path[i].geom.material.color = {r:1,g:0,b:0};


}

function tree_init(q, id) {

    // create tree object
    var tree = {};

    tree.id = id
    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];
    tree.vertices[0].p_vertex = 0;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}


function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);

    vertex.geom = temp_mesh;
}





