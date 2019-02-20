#include <math.h>
#include <stdint.h>   // for uint8_t and friends
#include <kilombo.h>

#include "functional_morphogenesis.h"
#include "util.h"

REGISTER_USERDATA(USERDATA)

#define MOTORS 1 // Whether motors are switched on. Values: 1: ON, 0: OFF

#ifdef SIMULATOR
//#include "readjson.h"
#else
#include <avr/io.h>  // for microcontroller register defs

#endif


#define COMM_R 85               // Communication range
#define DIFF_R 85               // Diffusion range
#define POLAR_TH 4.0            // Threshold for molecule u colour gradient
#define POLAR_TH_TASK_ALLOC 3.0 // Threshold to become polarized    
#define EDGE_TH 0.87            // Ratio between the average number of neighbors of the robot and the average number of neighbors' neighbors for edge detection
#define ALPHA_TH 0.01           // Ratio of average update
#define DIST_CRIT 45            // Distance that a robot is considered to be close
#define R2 120                  // For probabilistic purposes
#define COUNTER_WAIT 8000       // kilo_ticks to wait when the robot tries to orbit but there is another robot orbiting in the area

// Types of messages
#define MSG_TYPE_MORPHOGEN 0
#define MSG_TYPE_MOLECULES 1


// Parameters for the linear reaction-diffusion model
#define A_VAL 0.08
#define B_VAL -0.08
#define C_VAL 0.03
#define D_VAL 0.03
#define E_VAL 0.1
#define F_VAL 0.12
#define G_VAL 0.06
#define D_u 0.5   
#define D_v 10
#define LINEAR_R 160
#define SYNTH_U_MAX 0.23
#define SYNTH_V_MAX 0.5
#define DT 0.00005


// Parameters for local gradients 
#define MAX_HOPS_TO_MORPHOGEN_GRADIENT 1 // max included
#define MIN_NUMBER_ROBOTS_POLARIZED_FOR_POLARIZATION 3// min included
#define MIN_NUMBER_ROBOTS_POLARIZED_FOR_STOPPING 5 // min included 

#define MAX_DISTANCE_TO_POLARIZED_NEIGHBOURS_FOR_POLARIZATION 85 // max included


#ifdef SIMULATOR // In simulation

    // Parameters to control self-organised transition from patterning to exploration
    #define KILO_TICKS_START_COUNTING_DIFF 67000
    #define KILO_TICKS_COUNTING_WINDOW 8000
    #define MAX_DIFF_CONCENTRATION 1.5
    #define MIN_NEIGH_PATTERN_ACTIVE 2
    
    // Controls how long a robot must be orbiting to become source after stopping. Helps with sudden orbiting robots
    #define MAX_COUNTER_ORBITING 1000 

    // Controls how many steps the robots wait before becoming unpolarized
    #define MAX_COUNTER_POLARIZED 3000 
     
    // Parameters for improving edge detection to reduce the number of robots erroneously detected on the edge (in the inside, for example)
    #define MIN_N_NEIGHBOURS_ON_THE_EDGE 0
    #define MIN_N_POLARIZED_NEIGHBOURS_ON_THE_EDGE 0

    // Parameter that controls the probability to become source after losing the source in the cluster
    #define MAX_NUMBERS_FOR_SOURCE_PROBABILITY 4

    // Parameters for preventing faulty-motion robots erroneously detected on the edge from moving after a while
    #define DIFFERENT_NEIGHBORS_TH 0
    #define MIN_N_NEIGHBOURS_FOLLOW 0
    #define MAX_TIMES_ORBITING_WITH_SAME_NEIGH 100000
  
  
#else // In real robots
    
    // Parameters to control self-organised transition from patterning to exploration
    #define KILO_TICKS_START_COUNTING_DIFF 16000
    #define KILO_TICKS_COUNTING_WINDOW 2000
    #define MAX_DIFF_CONCENTRATION 1.5
    #define MIN_NEIGH_PATTERN_ACTIVE 2
    
    // Controls how long a robot must be orbiting to become source after stopping. Helps with sudden orbiting robots
    #define MAX_COUNTER_ORBITING 10000

    // Controls how many steps the robots wait before becoming unpolarized
    #define MAX_COUNTER_POLARIZED 30000 
    
    // Parameters for improving edge detection to reduce the number of robots erroneously detected on the edge (in the inside, for example)
    #define MIN_N_NEIGHBOURS_ON_THE_EDGE 1
    #define MIN_N_POLARIZED_NEIGHBOURS_ON_THE_EDGE 1

    // Parameter that controls the probability to become source after losing the source in the cluster
    #define MAX_NUMBERS_FOR_SOURCE_PROBABILITY 4

    // Parameters for preventing faulty-motion robots erroneously detected on the edge from moving after a while
    #define DIFFERENT_NEIGHBORS_TH 3
    #define MIN_N_NEIGHBOURS_FOLLOW 3
    #define MAX_TIMES_ORBITING_WITH_SAME_NEIGH 2
    
#endif


/*
 * Message rx callback function. It pushes message to ring buffer.
 */
void rxbuffer_push(message_t *msg, distance_measurement_t *dist) {
    received_message_t *rmsg = &RB_back();
    rmsg->msg = *msg;
    rmsg->dist = *dist;
    RB_pushback();
}


/*
 * Transmission of the message
 */
message_t *message_tx()
{
  if (mydata->message_lock)
    return 0;
  return &mydata->transmit_msg;
}


/*
 * A random byte is generated
 */
uint8_t rand_byte(){

	return rand_soft();

}


/*
 * Changes the state of the robot
 */
void set_bot_state(int state){

	mydata->bot_state = state;

}


/*
 * Returns the state of the robot
 */
int get_bot_state(void){

	return mydata->bot_state;

}


/*
 * Changes the type of motion that the robot performs
 */
void set_move_type(int type){

	mydata->move_type = type;

}


/*
 * Returns the type of motion the robot is performing
 */
int get_move_type(void){

	return mydata->move_type;

}


/*
 * Decodes the bit-wise byte received from neighbour with information from binary variables
 */
uint8_t decode_binary_information_from_neighbour(uint8_t i, uint8_t bit_number){

    return (0x01 & (mydata->neighbors[i].binary_information >> bit_number));
    
}

/*
 * Returns whether the robot has any neighbor with the specified ID
 */
uint8_t has_neighbour_with_id(uint16_t id){

	uint8_t i;
	uint8_t flag = 0;

	for(i = 0; i < mydata->N_Neighbors; i++){

		if(mydata->neighbors[i].ID == id){
			flag = 1;
			break;
		}

	}

	return flag;
}


/*
 * Returns whether the robot is polarized by threshold, i.e. concentration of molecule U is higher than POLAR_TH_TASK_ALLOC
 */
uint8_t polarized_by_polar_threshold(){

	uint8_t flag = 0;

	if(mydata->molecules_concentration[0] > POLAR_TH_TASK_ALLOC){
		flag = 1;

	}

	return flag;

}


/*
 * Returns whether the robot is polarized by patterning, i.e. it was polarized by threshold after patterning
 */
uint8_t polarized_by_patterning(){

	return (mydata->polarized_by_patterning == 1);

}


/*
 * Returns whether the robot is polarized by neighbours, i.e. it is within the range or polarization by neighbours and it is allowed
 */
uint8_t polarized_by_neighbours(){

	return (mydata->polarized_by_neighbours == 1);

}


/*
 * Returns whether the robot has at least the specified number of polarized neighbors
 */
uint8_t has_at_least_n_polarized_N(uint8_t n){

	uint8_t i;
	uint8_t count = 0;
	uint8_t flag = 0;

	for(i = 0; i < mydata->N_Neighbors; i++){

		if(decode_binary_information_from_neighbour(i, 0)){
			count++;
			
		}

		if(count >= n){
			flag = 1;
			break;
		}

	}

	return flag;

}


/*
 * Returns whether the robot has at least the specified number of polarized neighbors on the edge
 */
uint8_t has_at_least_n_polarized_N_on_the_edge(uint8_t n){

	uint8_t i;
	uint8_t count = 0;
	uint8_t flag = 0;

	for(i = 0; i < mydata->N_Neighbors; i++){

		if(decode_binary_information_from_neighbour(i, 0) && decode_binary_information_from_neighbour(i, 2)){
			count++;
			
		}

		if(count >= n){
			flag = 1;
			break;
		}

	}

	return flag;

}


/*
 * Returns whether the robot has at least the specified number of neighbors on the edge
 */
uint8_t has_at_least_n_N_on_the_edge(uint8_t n){

	uint8_t i;
	uint8_t count = 0;
	uint8_t flag = 0;

	for(i = 0; i < mydata->N_Neighbors; i++){

		if(decode_binary_information_from_neighbour(i, 2)){
			count++;
			
		}

		if(count >= n){
			flag = 1;
			break;
		}

	}

	return flag;

}


/*
 * Returns the distance to the nearest polarized-by-neighbours neighbor
 */
uint8_t get_dist_to_nearest_polarized(){

	uint8_t i;
	uint8_t dist = 255;

	for(i = 0; i < mydata->N_Neighbors; i++){

		if(decode_binary_information_from_neighbour(i, 0)){

			if(mydata->neighbors[i].dist  < dist) 
		        dist = mydata->neighbors[i].dist;

		}

	}

	return dist;

}


/*
 * In mode 1, it returns the closest neighbour with at least MIN_N_NEIGHBOURS_FOLLOW
 * In mode 2, it returns the closest neighbour with at least 1 neighbour
 */
uint16_t find_nearest_N_id(uint8_t mode){

	uint8_t i;
	uint16_t id = 0;
	uint8_t dist = 255;
	uint8_t min_n_neighbors;
	
	// Mode 1: the closest neighbour with at least MIN_N_NEIGHBOURS_FOLLOW
	if(mode == 1){
	    
	    min_n_neighbors = MIN_N_NEIGHBOURS_FOLLOW;
	    
	}
	
	// Mode 2: the closest neighbour with at least 1 neighbour
	else{
	
	    min_n_neighbors = 1;
	
	}
	

    for(i = 0; i < mydata->N_Neighbors; i++){

	    if(mydata->neighbors[i].dist  < dist && mydata->neighbors[i].N_Neighbors > min_n_neighbors) {
		    id = mydata->neighbors[i].ID;
		    dist = mydata->neighbors[i].dist;
	    }

    }
	    
	return id;
}


/*
 * Returns the distance to the furthest neighbor
 */
uint16_t find_most_distant_N_id(){

	uint8_t i;
	uint16_t id = 0;
	uint8_t dist = 0;

	for(i = 0; i < mydata->N_Neighbors; i++){

		if(mydata->neighbors[i].dist  > dist) {
			id = i;
			dist = mydata->neighbors[i].dist;
		}

	}

	return id;
}


/*
 * Returns the distance of the neighbor with the specified ID
 */
uint8_t get_dist_by_id(uint16_t id){

	uint8_t i;
	uint8_t dist = 255;

	for(i = 0; i < mydata->N_Neighbors; i++){

		if(mydata->neighbors[i].ID == id){
			dist = mydata->neighbors[i].dist;
			break;
		}

	}

	return dist;

}


/*
 * Returns the state of the neighbor with the specified ID
 */
uint8_t get_state_by_id(uint16_t id){

	uint8_t i;
	uint8_t state = WAIT;

	for(i = 0; i < mydata->N_Neighbors; i++){

		if(mydata->neighbors[i].ID == id){
			state = mydata->neighbors[i].n_bot_state;
			break;
		}

	}

	return state;

}


/*
 * Returns the difference in distance with respect to the last update of the neighbor with the specified ID
 */
int get_diff_dist_by_id(uint16_t id){

	uint8_t i;
	int diff_dist = 255;

	for(i = 0; i < mydata->N_Neighbors; i++){

		if(mydata->neighbors[i].ID == id){
			diff_dist = mydata->neighbors[i].delta_dist;
			break;
		}

	}

	return diff_dist;

}

/*
 *  Returns true if at least one neighbor is in ORBIT state
 */
uint8_t check_orbit_state(){

	uint8_t i;
	uint8_t flag = 0;

	for(i = 0; i < mydata->N_Neighbors; i++){

		if(mydata->neighbors[i].n_bot_state == ORBIT) {
			flag = 1;
			break;
		}

	}

	return flag;

}


/*
 *  Returns true if all neighbors are in WAIT state
 */
uint8_t check_wait_state(){

	uint8_t i;
	uint8_t flag = 1;

	for(i = 0; i < mydata->N_Neighbors; i++){

/*		if(mydata->neighbors[i].n_bot_state == WAIT){*/
		if(mydata->neighbors[i].n_bot_state == ORBIT || mydata->neighbors[i].n_bot_state == FOLLOW){
			flag = 0;
			break;
		}

	}

	return flag;

}


/*
 * Returns true if all neighbors or at least a minimum number of them aren't in PATTERN_FORMATION or PATTERN_CHECKING state
 *      - allNeighbours: 1 if all neighbours should be counted, or 0 if there should be at least MIN_NEIGH_PATTERN_ACTIVE neighbours
 */
uint8_t phase_transition(uint8_t allNeighbours){

	uint8_t i;

    if(allNeighbours){

        uint8_t flag = 1;

        for(i = 0; i < mydata->N_Neighbors && flag; i++){

	        if(mydata->neighbors[i].n_bot_state == PATTERN_FORMATION || mydata->neighbors[i].n_bot_state == PATTERN_CHECKING){
	            flag = 0;
	        }
        }

        return flag;
    }
    else{

        uint8_t count = 0;

        for(i = 0; i < mydata->N_Neighbors && count <= MIN_NEIGH_PATTERN_ACTIVE; i++){

	        if(!(mydata->neighbors[i].n_bot_state == PATTERN_FORMATION || mydata->neighbors[i].n_bot_state == PATTERN_CHECKING)){
	            count++;
	        }
        }

        return count >= MIN_NEIGH_PATTERN_ACTIVE;

    }
}


/*
 * Computes the average number of neighbors' neighbors taking into account the distance to them
 */
float calc_avg_NNs(){

	float sum = 0;
	float w_sum = 0;
	uint8_t i;


	for(i = 0; i < mydata->N_Neighbors; i++){

			float w = 1/(float) mydata->neighbors[i].dist;
			sum = sum + w * (float) mydata->neighbors[i].N_Neighbors;
			w_sum = w_sum + w;

	}

	return sum/w_sum;

}


/*
 * Function to compute a running average by weighing previous and current observations
 */
float calc_apprx_running_avg(float old_avg, float num, float alpha){

	float avg;

	avg = alpha * num + (1 - alpha) * old_avg;

	return avg;

}


/*
 * Concentration of U and V is updated based on the linear model for reaction-diffusion 
 */
void regulation_linear_model(){

    // D_u and D_v
	float D[2];
	D[0] = D_u;
	D[1] = D_v;

    // R
	float linear_R = LINEAR_R;

	// Laplace operator for U and V
	float lap[2];
	lap[0] = 0;
	lap[1] = 0;

    // All neighbors contribute the same
    float weight = 1.0;

    // The Laplace operator is calculated for U and V
    int i;
	for (i = 0; i < mydata->N_Neighbors; i++)
	{
        // Only neighbors nor further apart than DIFF_R (currently 85 mm) and in patterning phase
        if(mydata->neighbors[i].dist <= DIFF_R && ( mydata->neighbors[i].n_bot_state == PATTERN_FORMATION || 
                                                    mydata->neighbors[i].n_bot_state == PATTERN_CHECKING)){
          lap[0] += weight * (mydata->neighbors[i].molecules_concentration[0] - mydata->molecules_concentration[0]);
          lap[1] += weight * (mydata->neighbors[i].molecules_concentration[1] - mydata->molecules_concentration[1]);
        }
	}

    // Maximum production
	float synth_u_max = SYNTH_U_MAX;
	float synth_v_max = SYNTH_V_MAX; 

    // These variables will have f(u,v) and g(u,v) eventually
	float synth_rate_u;
	float synth_rate_v;

	synth_rate_u = (A_VAL * mydata->molecules_concentration[0] + B_VAL * mydata->molecules_concentration[1] + C_VAL);
	synth_rate_v = (E_VAL * mydata->molecules_concentration[0] - F_VAL);

	if(synth_rate_u < 0) synth_rate_u = 0;
	if(synth_rate_u >= synth_u_max) synth_rate_u = synth_u_max;

	if(synth_rate_v < 0) synth_rate_v = 0;
	if(synth_rate_v >= synth_v_max) synth_rate_v = synth_v_max;

	synth_rate_u = synth_rate_u - D_VAL * mydata->molecules_concentration[0];
	synth_rate_v = synth_rate_v - G_VAL * mydata->molecules_concentration[1];

    // Rate of change in the concentration
	float dG[2];
	dG[0] = linear_R * synth_rate_u + D[0] * lap[0];
	dG[1] = linear_R * synth_rate_v + D[1] * lap[1];

	// Update of the concentration 
	float dt = DT; 
	mydata->molecules_concentration[0] += dt * dG[0];
	mydata->molecules_concentration[1] += dt * dG[1];
	
}


/*
 * Returns whether the robot is on the edge based on the ratio of its neighbors and its neighbors' neighbors
 */
uint8_t edge_prob_running_avg_ratio_NNs(){

	uint8_t  prob;
	float diff_th = EDGE_TH;


	if(mydata->running_avg_Ns/mydata->running_avg_NNs < diff_th) prob = 1;
	else prob = 0;

	return prob;

}



/*
 * Returns whether the robot is on the edge based on the ratio of its neighbors and its neighbors' neighbors
 */
uint8_t test_edge(){	

    return edge_prob_running_avg_ratio_NNs();

}


/*
 * Gerenates a random ID
 */
uint16_t reset_id(){

	uint16_t id;
	id = (rand_byte() << 8) | rand_byte();

	return id;

}


/*
 * Returns the minimum number of hops to the source of the morphogen gradient from my neighbours
 */
uint8_t min_neighbour_hops_to_morphogen_gradient()
{

    int i;
    uint8_t min = 255;
    for(i = 0; i < mydata->N_Neighbors; i++){

		if(mydata->neighbors[i].hops_to_morphogen_gradient < min){
			min = mydata->neighbors[i].hops_to_morphogen_gradient;
		}

	}

    return min;

}


/* 
 * A bot becomes polarized by neighbours when it stops by polarized robots and 
 * there are are least MIN_NUMBER_ROBOTS_POLARIZED_FOR_POLARIZATION polarized neighbours
 * in range MAX_DISTANCE_TO_POLARIZED_NEIGHBOURS_FOR_POLARIZATION
 */
uint8_t test_polarized_by_neighbours(){

    return (get_dist_to_nearest_polarized() <= MAX_DISTANCE_TO_POLARIZED_NEIGHBOURS_FOR_POLARIZATION && 
            has_at_least_n_polarized_N(MIN_NUMBER_ROBOTS_POLARIZED_FOR_POLARIZATION));

}



/*
 *  Returns true if all my neighbours with valid hops have me as source of the morphogen gradient
 */
uint8_t check_all_neighbours_have_my_source_id(){

	uint8_t i;
	uint8_t n_neighbours_with_valid_hops = 0;
    uint8_t flag_different_id = 0;

	for(i = 0; i < mydata->N_Neighbors; i++){

        // Neighbour hsa valid hops
		if(mydata->neighbors[i].hops_to_morphogen_gradient != 127 && mydata->neighbors[i].hops_to_morphogen_gradient != -1){

            n_neighbours_with_valid_hops++;

            // Neighbour has me as source
            if(mydata->neighbors[i].source_signal_id != mydata->source_signal_id) {
			    flag_different_id = 1;
    			break;  
            }
		}

	}

	return n_neighbours_with_valid_hops != 0 && flag_different_id == 0;
}


/*
 *  Returns:
 *      - 0: if all neighbours have hops = 127
 *      - 1: if neighbours with hops <= MAX_HOPS_MORPHOGEN_GRADIENT have the same source
 *      - 2: if neighbours with hops <= MAX_HOPS_MORPHOGEN_GRADIENT have different ids
 *      - 3: if no neighbours with source id (all neighbours with hops = 127 or -1)
 */
uint8_t check_all_neighbours_have_same_source_id(){

	uint8_t i;
	uint8_t flag = 0;
    uint16_t id = 0;

	for(i = 0; i < mydata->N_Neighbors; i++){

        if(mydata->neighbors[i].hops_to_morphogen_gradient <= MAX_HOPS_TO_MORPHOGEN_GRADIENT && mydata->neighbors[i].hops_to_morphogen_gradient != -1){

            // First ocurrence
            if(flag == 0){
    
                flag = 1;
                id = mydata->neighbors[i].source_signal_id;

            } 
    
            if(flag == 1 && mydata->neighbors[i].source_signal_id != id){

                flag = 2;
                break;

            }

        }
	}

    if(flag == 0){

        for(i = 0; i < mydata->N_Neighbors; i++){

            if(mydata->neighbors[i].hops_to_morphogen_gradient == -1){
                flag = 3;
                break;
            }

        }

    }

	return flag;

}


/* 
 * Returns the source id of the neighbours with valid hops.
 * IMPORTANT: it assumes all neighbours with valid hops have the same id. That only happens if check_all_neighbours_have_same_source_id() == 1
 */
uint16_t get_source_id_from_neighbours(){

	uint8_t i;
    uint16_t id = 0;

	for(i = 0; i < mydata->N_Neighbors; i++){

        if(mydata->neighbors[i].hops_to_morphogen_gradient != 127 && mydata->neighbors[i].hops_to_morphogen_gradient != -1){

            id = mydata->neighbors[i].source_signal_id;
            break;

        }
    }

    return id;
}


/*
 * Returns the minimum number of hops from neighbours with the source id given
 */
uint8_t min_hops_to_neighbours_with_source_id(uint16_t id){

    uint8_t min_hops = 127;
    uint8_t i;

	for(i = 0; i < mydata->N_Neighbors; i++){

        if(mydata->neighbors[i].source_signal_id == id && mydata->neighbors[i].hops_to_morphogen_gradient < min_hops){

            min_hops = mydata->neighbors[i].hops_to_morphogen_gradient;
        }
    }

    return min_hops;
}


/*
 * Returns the id of the other source emitting a signal different to mine
 */
uint16_t find_source_signal_id_different_to_mine(){

	uint8_t i;
    uint16_t id = 0;

	for(i = 0; i < mydata->N_Neighbors; i++){

        if(mydata->neighbors[i].hops_to_morphogen_gradient != 127 && mydata->neighbors[i].hops_to_morphogen_gradient != -1 && (mydata->neighbors[i].source_signal_id != mydata->source_signal_id)){

            id = mydata->neighbors[i].source_signal_id;
            break;

        }
    }

    return id;

}


/*
 * Returns whether the robot should stop being source if there are other new sources in the cluster and this robot doesn't have the highest ID
 */
uint8_t check_for_becoming_not_source(){

    uint8_t flag = 0;
    uint8_t i;

    for(i = 0; i < mydata->N_Neighbors; i++){

        if(mydata->neighbors[i].hops_to_morphogen_gradient == 0 && decode_binary_information_from_neighbour(i, 1) && kilo_uid < mydata->neighbors[i].ID){

            flag = 1;
            break;

        }
    }


    return flag;
}



/*
 * Local gradients algorithm
 */
void wait(){

    uint8_t r1, r2; // For random numbers

    // Only if it's allowed to become polarized, the robot executes the local gradients algorithm
    if(mydata->allowed_to_be_polarized_by_neighbours == 1){

        if(polarized_by_patterning()){

            mydata->polarized_by_neighbours = 1;      // Task allocation
            mydata->hops_to_morphogen_gradient = -1;  // It's used to be seen by neighbours. When there are enough, they will pass the test_polarized_by_neighbours() check

            mydata->counter = MAX_COUNTER_POLARIZED; // Counter reset

        }
          
        /* 
         *  A bot becomes polarized by neighbours when it stops by
         *  polarized robots and there are enough polarized neighbours 
        */
        if(test_polarized_by_neighbours()){
            
            if(mydata->hops_to_morphogen_gradient != 0){ // This robot isn't source

                // From not source to source
                if(mydata->new_source == 1){

                    mydata->hops_to_morphogen_gradient = 0;
                    mydata->source_signal_id = kilo_uid;

                }

                // Continues being not source
                else{

                    if(mydata->new_source == 0){ // There isn't a new source in the cluster

                        switch(check_all_neighbours_have_same_source_id()){

                            case 0: // All neighbours have hops = 127

                                r1 = rand_byte();
                                r2 = rand_byte();

                                // If I'm on the edge and there isn't a source in the cluster, I might become one with probability (MAX_NUMBERS_FOR_SOURCE_PROBABILITY / 256)^2
                                if(test_edge() && r1 < MAX_NUMBERS_FOR_SOURCE_PROBABILITY && r2 < MAX_NUMBERS_FOR_SOURCE_PROBABILITY)
                                    mydata->new_source = 1;

                                break;

                            case 1: // All neighbours with hops < 127 have the same source id

                                // Min number of hops to the source of the signal in the cluster
                                mydata->source_signal_id = get_source_id_from_neighbours();
                                mydata->hops_to_morphogen_gradient = min_hops_to_neighbours_with_source_id(mydata->source_signal_id);
                                if(mydata->hops_to_morphogen_gradient < 127 && mydata->hops_to_morphogen_gradient != -1)
                                    mydata->hops_to_morphogen_gradient++;

                                break;

                            case 2: // All neighbours with hops < 127 have different source ids

                                // Min number of hops to the different source of the signal to the previous one I had
                                mydata->source_signal_id = find_source_signal_id_different_to_mine();
                                mydata->hops_to_morphogen_gradient = min_hops_to_neighbours_with_source_id(mydata->source_signal_id);
                                if(mydata->hops_to_morphogen_gradient < 127 && mydata->hops_to_morphogen_gradient != -1)
                                    mydata->hops_to_morphogen_gradient++;

                                break;
                        }
                    }
                    else if(mydata->new_source == 2){ // There is a new source in the cluster

                        // It calculates the min hops to the new source in the cluster until all neighbours have it as source
                        mydata->hops_to_morphogen_gradient = min_hops_to_neighbours_with_source_id(mydata->source_signal_id);
                        if(mydata->hops_to_morphogen_gradient < 127 && mydata->hops_to_morphogen_gradient != -1)
                            mydata->hops_to_morphogen_gradient++;

                        if(check_all_neighbours_have_my_source_id())
                            mydata->new_source = 0;
                    }
                }
            }       
            else{ // This robot is source

                // It isn't a new source in the cluster
                if(mydata->new_source == 0){

                    // If there is at least one neighbour with another source, it means a new source has arrived to the cluster
                    if(!check_all_neighbours_have_my_source_id()){

                        // Min number of hops to the new source of the signal
                        mydata->source_signal_id = find_source_signal_id_different_to_mine();
                        mydata->hops_to_morphogen_gradient = min_hops_to_neighbours_with_source_id(mydata->source_signal_id);
                        if(mydata->hops_to_morphogen_gradient < 127 && mydata->hops_to_morphogen_gradient != -1)
                            mydata->hops_to_morphogen_gradient++;
                    }
                }
               
                // It's a new source in the cluster
                if(mydata->new_source == 1){

                    // When all neighbours have me as source, I continue to be source, but not a new source anymore
                    if(check_all_neighbours_have_my_source_id()){

                        mydata->new_source = 0;                            

                    }
                    else{

                        // True if there's another new source in the cluster, then it becomes not source anymore if this robot doesn't have the highest ID
                        if(check_for_becoming_not_source()){

                            mydata->new_source = 0;
                            mydata->hops_to_morphogen_gradient = 127;

                        }

                        // No more sources in the cluster. Just waiting for all my neighbours to update their information about me. Keeps being source
                        else{

                        }
                    }
                }
            }

            // If the robot is outside the range of maximum hops to the source
            if(mydata->hops_to_morphogen_gradient > MAX_HOPS_TO_MORPHOGEN_GRADIENT){

                mydata->hops_to_morphogen_gradient = 127;

                // Reduces the counter to finally become unpolarized
                if(mydata->counter > 0)
                    mydata->counter--;

                if(polarized_by_patterning())
                    mydata->polarized_by_patterning = 0;

            }

            // Inside the range, i.e., polarized
            else{

                mydata->polarized_by_neighbours = 1;
                mydata->polarized_by_patterning = 0;
                mydata->counter = MAX_COUNTER_POLARIZED; // Counter reset

            }
        }

        // There aren't enough polarized-by-neighbours neighbours
        else{

            // Only when it's been polarized by neighbours, it reduces the counter to finally become unpolarized. 
            // This prevents losing the polarized-by-patterning clusters
            if(!polarized_by_patterning()){

                if(mydata->counter > 0)
                    mydata->counter--;

            }

        }

        // If counter reaches zero, it finally becomes unpolarized
        if(mydata->counter == 0){

            mydata->polarized_by_neighbours = 0;
            mydata->hops_to_morphogen_gradient = 127;
            mydata->new_source = 0;

        }
    }
}


/*
 * The robot rotates in one direction until it starts being further apart from the nearest robot, then switches direction.
 */
void move_by_turning(int gradient){

	if(mydata->move_switch_flag == 0){

        // If getting closer, perhaps change direction of rotation
		if(gradient < 0) mydata->move_switch_flag = 1;

        // Getting further, keep rotating in the same direction
		if(gradient > 0) return;

	}


    // Getting closer with the current direction
	else{

        // If getting further, change direction of rotation
		if(gradient > 0) {

			mydata->move_switch_flag = 0;

			switch (get_move_type()) {
				case RIGHT:
					motion(LEFT);
					set_move_type(LEFT);
					break;
				case LEFT:
					motion(RIGHT);
					set_move_type(RIGHT);
					break;
				default:
					break;
			}

		}

	}


}


/*
 * The robot orbits around its neighbors trying to maintain a constant distance defined by dist_th by switching between left and right motion
 */
void orbit2(uint8_t dist, uint8_t dist_th){

	if(dist <= dist_th - 1){

		if(mydata->last_turn == RIGHT){
			motion(LEFT);
			set_move_type(LEFT);
		}
		else{
			motion(RIGHT);
			set_move_type(RIGHT);
		}

	}

	if(dist >= dist_th){

		if(mydata->last_turn == RIGHT){
			motion(RIGHT);
			set_move_type(RIGHT);
		}
		else{
			motion(LEFT);
			set_move_type(LEFT);
		}
	}

}



/* 
 * It processes a received message at the front of the ring buffer.
 * It goes through the list of neighbors. If the message is from a bot
 * already in the list, it updates the information, otherwise
 * it adds a new entry to the list
 */
void process_message()
{
  uint8_t i,j;
  uint16_t ID;

  uint8_t msg_type = RB_front().msg.type;
  uint8_t *data = RB_front().msg.data;
  ID = data[0] | (data[1] << 8);
  uint8_t d = estimate_distance(&RB_front().dist);
  

  if(d > COMM_R && mydata->N_Neighbors > 0 && get_bot_state() != FOLLOW) return;

  // search the neighbor list by ID
  for (i = 0; i < mydata->N_Neighbors; i++)
    if (mydata->neighbors[i].ID == ID){// found it
    	mydata->neighbors[i].delta_dist = d - mydata->neighbors[i].dist;
    	break;
    }

  if (i == mydata->N_Neighbors)
      {  // this neighbor is not in list
        if (mydata->N_Neighbors < MAXN-1) // neighbor list is not full
     	   mydata->N_Neighbors++;
        else
            i = find_most_distant_N_id(); // overwrite the most distant neighbor

        mydata->neighbors[i].delta_dist = 0;

      }

  // i now points to where this message should be stored
  mydata->neighbors[i].ID = ID;
  mydata->neighbors[i].timestamp = kilo_ticks;
  mydata->neighbors[i].dist = d;
  mydata->neighbors[i].N_Neighbors = data[2];
  mydata->neighbors[i].n_bot_state = data[7];


  if(msg_type == MSG_TYPE_MOLECULES){
      uint8_t signo_rec;
      uint8_t exp_rec;
      uint16_t mant_rec;
      uint16_t bit1;
      float mant_fl;

      signo_rec=0;
      exp_rec=0;
      mant_rec=0;
      bit1=0;


      int jj;


      for (j = 0; j < 2; j++){

	    // recover from "half" precision
	    signo_rec = data[3+j] >> 7;
	    exp_rec = (data[3+j] >> 2) & 0x1F;
	    mant_rec = ((data[3+j] & 0x3) << 8) | data[3+2+j];

	    mant_fl=0;
	    for (jj=9; jj>=0; jj--){

	      bit1 = mant_rec>>jj;

	      mant_fl = mant_fl + bit1*pow(2,jj-10);

	      mant_rec = mant_rec - bit1*pow(2,jj);

	    }



	    if(exp_rec==31 && signo_rec==0)  mydata->neighbors[i].molecules_concentration[j]= 65504;

	    else if (exp_rec==31 && signo_rec==1) mydata->neighbors[i].molecules_concentration[j]= -65504;

	    else if (exp_rec==-15 && mant_rec==0) mydata->neighbors[i].molecules_concentration[j] = 0;

	    else if (exp_rec==-15 && mant_rec!=0) mydata->neighbors[i].molecules_concentration[j] = (float) pow(-1,signo_rec)*pow(2,exp_rec-15+1)*(0+mant_fl);

	    else mydata->neighbors[i].molecules_concentration[j] = (float) pow(-1,signo_rec)*pow(2,exp_rec-15)*(1+mant_fl);

      }

      mydata->neighbors[i].binary_information = 0;
      mydata->neighbors[i].hops_to_morphogen_gradient = 127;
      mydata->neighbors[i].source_signal_id = 0;

  }
  else if(msg_type == MSG_TYPE_MORPHOGEN){

    mydata->neighbors[i].molecules_concentration[0] = -1;
    mydata->neighbors[i].molecules_concentration[1] = -1;

    mydata->neighbors[i].hops_to_morphogen_gradient = data[3];
    mydata->neighbors[i].binary_information = data[6];
    mydata->neighbors[i].source_signal_id = data[4] | (data[5] << 8);

  }
}


/*
 * This function:
 *   - Processes all messages received since the last time
 *   - Updates neighbors table
 *   - Updates Number of neighbors and Number of neighbors' neighbors running averages
 */
void receive_inputs()
{

    // Processes all messages received since the last time the bot read them (erased after reading)    
    while (!RB_empty()) {
        process_message();
        RB_popfront();
    }

    mydata->running_avg_Ns = calc_apprx_running_avg(mydata->running_avg_Ns, mydata->N_Neighbors, ALPHA_TH);

    if(mydata->N_Neighbors > 0){
      mydata->running_avg_NNs = calc_apprx_running_avg(mydata->running_avg_NNs, calc_avg_NNs(), ALPHA_TH);
    }
    else mydata->running_avg_NNs = calc_apprx_running_avg(mydata->running_avg_NNs, 0, ALPHA_TH);

}



/* 
 * Goes through the list of neighbors and removes entries older than a threshold, currently 2 seconds.
 */
void purgeNeighbors(void)
{
  int8_t i;

  for (i = mydata->N_Neighbors-1; i >= 0; i--)
      if (kilo_ticks - mydata->neighbors[i].timestamp  > 64)
      { //this one is too old.
        mydata->neighbors[i] = mydata->neighbors[mydata->N_Neighbors-1]; //replace it by the last entry
        mydata->N_Neighbors--;
      }
}


/*
 * The message is updated to reflect changes in state, concentration, number of neighbors and ID
 */
void setup_message(void)
{
  mydata->message_lock = 1;  //don't transmit while we are forming the message
  mydata->transmit_msg.type = MSG_TYPE_MOLECULES;
  mydata->transmit_msg.data[0] = kilo_uid & 0xff; //0 low  ID
  mydata->transmit_msg.data[1] = kilo_uid >> 8;   //1 high ID

  mydata->transmit_msg.data[2] = mydata->N_Neighbors;     //2 number of neighbors
  mydata->transmit_msg.data[7] = get_bot_state();

  int i;

  //2 half-prec + 2 single-prec


  uint8_t signo;
  uint8_t exp;
  uint32_t mant;
  uint8_t exp_h;
  uint16_t mant_h;
  uint8_t byte_1;
  uint8_t byte_2;
  int exp_real;
  long fl;


  for (i = 0; i < 2; i++){

	  fl = *(long*)&mydata->molecules_concentration[i];    //mydata->G[i];
	  signo = fl >> 31;
	  exp = (fl >>23) & 0xff;
	  mant =  fl & 0x7FFFFF;

	  // Convert to "half" precision

	  exp_real = exp-127;

	  //cut the size of the exponent
	  if(exp_real<-15) exp_real=-15;
	  else if(exp_real>16) exp_real=16;

	  exp_h= exp_real + 15;
	  mant_h = mant >> 13;

	  byte_1 = signo << 7 | exp_h << 2 | mant_h >> 8;
	  byte_2 = mant_h & 0xff;

	  mydata->transmit_msg.data[3+i] = byte_1;
	  mydata->transmit_msg.data[3+2+i] = byte_2;

  }

  mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);
  mydata->message_lock = 0;
}


/*
 * The message is updated to reflect changes
 */
void  setup_message_morphogen_gradient()
{
  mydata->message_lock = 1;  //don't transmit while we are forming the message
  mydata->transmit_msg.type = MSG_TYPE_MORPHOGEN;

  mydata->transmit_msg.data[0] = kilo_uid & 0xff; //0: low  ID
  mydata->transmit_msg.data[1] = kilo_uid >> 8;   //1: high ID

  mydata->transmit_msg.data[2] = mydata->N_Neighbors;     //2: number of neighbors

  mydata->transmit_msg.data[3] = mydata->hops_to_morphogen_gradient;   //3: hops to source

  mydata->transmit_msg.data[4] = mydata->source_signal_id & 0xff; //4: low  ID
  mydata->transmit_msg.data[5] = mydata->source_signal_id >> 8;   //5: high ID

  /*
   * 6: Binary information:
        First bit (position 0): polarized by neighbours
        Second bit (position 1): new_source
        Third bit (position 2): test_edge
   */
  mydata->transmit_msg.data[6] = (0x00) | (test_edge() & 0x01); 
  mydata->transmit_msg.data[6] = mydata->transmit_msg.data[6] << 1; 
  mydata->transmit_msg.data[6] = (mydata->transmit_msg.data[6] & 0xfe) | ((mydata->new_source > 0) & 0x01);   
  mydata->transmit_msg.data[6] = mydata->transmit_msg.data[6] << 1;
  mydata->transmit_msg.data[6] = (mydata->transmit_msg.data[6] & 0xfe) | (polarized_by_neighbours() & 0x01); 

  mydata->transmit_msg.data[7] = get_bot_state(); //7: State

  mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);
  mydata->message_lock = 0;
}

/*
 * Returns whether the robot should transit from WAIT state to FOLLOW state. 
 */
uint8_t wait_to_follow(){

	uint8_t flag;

	uint16_t id_nearest = find_nearest_N_id(1);
	uint8_t dist_nearest = get_dist_by_id(id_nearest);


	if(
	    mydata->allowed_to_be_polarized_by_neighbours <= (2 + MAX_TIMES_ORBITING_WITH_SAME_NEIGH)
	    && test_edge()
	    && get_state_by_id(id_nearest) == WAIT
	    && dist_nearest > DIST_CRIT + 15
	    && mydata->N_Neighbors > 0
	    && has_at_least_n_N_on_the_edge(MIN_N_NEIGHBOURS_ON_THE_EDGE)
	) flag = 1;
	else flag = 0;

	return flag;
}


/*
 * Returns whether the robot should transit from WAIT state to ORBIT state.
 */
uint8_t wait_to_orbit(){

	uint8_t flag;

    if(
        test_edge() 
        && has_at_least_n_N_on_the_edge(MIN_N_NEIGHBOURS_ON_THE_EDGE)
	    && check_wait_state() // True if all neighbours are in WAIT state
	    && !polarized_by_patterning()
        && mydata->hops_to_morphogen_gradient != 0
        && (!polarized_by_neighbours() || !has_at_least_n_polarized_N(1) || (has_at_least_n_polarized_N(1) && get_dist_to_nearest_polarized() > (DIST_CRIT)))
	    && mydata->counter == 0
	    && (get_dist_to_nearest_polarized() > (DIST_CRIT) || !has_at_least_n_polarized_N(1))
        && mydata->N_Neighbors != 0
	    ) flag = 1;
	else flag = 0;
	
	
	return flag;
}

uint8_t check_nearest_neighbour_moving(){

    uint16_t id_nearest = find_nearest_N_id(1);
	
	return (get_state_by_id(id_nearest) != WAIT);
}

/*
 * Returns whether the robot should transit from ORBIT state to WAIT state. 
 */
uint8_t orbit_to_wait(){

	uint8_t flag;

	uint16_t id_nearest = find_nearest_N_id(1);
	uint8_t dist_nearest = get_dist_by_id(id_nearest);

	if(
		get_state_by_id(id_nearest) != WAIT //nearest neighbour is moving
		|| dist_nearest > DIST_CRIT + 15// nearest neighbour is too far and cannot be reached by rotating (bot radius = 17)
		|| !test_edge() //bot no longer on the edge
		|| !has_at_least_n_N_on_the_edge(MIN_N_NEIGHBOURS_ON_THE_EDGE)
		|| (mydata->new_source >= 3 && get_dist_to_nearest_polarized() < (DIST_CRIT - 1) && has_at_least_n_polarized_N(MIN_NUMBER_ROBOTS_POLARIZED_FOR_STOPPING)) //bot close to a polarized
		|| mydata->N_Neighbors == 0
		|| (mydata->counter == (MAX_COUNTER_ORBITING + 1) && mydata->n_different_neighbors < DIFFERENT_NEIGHBORS_TH)
		) flag = 1;
	else flag = 0;
	
	
	return flag;
}


/*
 * Returns whether the robot should transit from FOLLOW state to WAIT state.
 */
uint8_t follow_to_wait(){

	uint8_t flag;

	uint16_t id_nearest = find_nearest_N_id(1);
	uint8_t dist_nearest = get_dist_by_id(id_nearest);

	if(
		get_state_by_id(id_nearest) != WAIT
		|| mydata->N_Neighbors == 0
		|| !has_at_least_n_N_on_the_edge(MIN_N_NEIGHBOURS_ON_THE_EDGE)
		|| dist_nearest <= DIST_CRIT
	) flag = 1;
	else flag = 0;

	return flag;
}


/*
 * Returns how many different neighbours there are compared when it started the orbiting block
 */
uint8_t count_n_different_neighbors(){

    uint8_t n_different_neighbors = 0;

    int i = 0;
    int j = 0;
    int found;
    
    for(i = 0; i < mydata->initial_n_neighbors; i++){
    
        found = 0;
        for(j = 0; j < mydata->N_Neighbors && !found; j++){

            if(mydata->initial_neighbors_ID[i] == mydata->neighbors[j].ID)
                found = 1;                            
        
        }
        
        if(!found)
            n_different_neighbors++;   
    }

    n_different_neighbors += mydata->N_Neighbors - i;
    
    return n_different_neighbors;
}


/*
 * Saves information about neighbours at the start of the orbiting block
 */
void reset_initial_neighbours(){

	int i;
	for(i = 0; i < mydata->N_Neighbors; i++)
	    mydata->initial_neighbors_ID[i] = mydata->neighbors[i].ID;
	    
    mydata->initial_n_neighbors = mydata->N_Neighbors;
}

/*
 * Manages the transitions between states
 */
void edge_flow(){

	switch (get_bot_state()) {

		case ORBIT:

            // If two orbiting robots are going to stop because they're orbiting around each one, 
            // delay a random time between 0 and 0.75 seconds, so that only one of them stops
            if(check_nearest_neighbour_moving()){
            
                delay(rand_byte()*3);
            
            }

			if(orbit_to_wait()){

                // Only if it has been orbiting for some time with different neighbours, it is allowed to be polarized and source. 
                // This prevents sudden changes in sources and polarization in the middle
                if(mydata->new_source >= 3 && test_polarized_by_neighbours() && has_at_least_n_polarized_N_on_the_edge(MIN_N_POLARIZED_NEIGHBOURS_ON_THE_EDGE)){
                
                    mydata->allowed_to_be_polarized_by_neighbours = 1;
                    mydata->new_source = 1;

                }
                else{

                    // If it has been orbiting for at least half an orbiting block, it checks whether it has changed neighbours, i.e., if it has really moved
                    if (( (mydata->counter >= (MAX_COUNTER_ORBITING/2)) && mydata->n_different_neighbors < DIFFERENT_NEIGHBORS_TH)) {
                        
                        mydata->allowed_to_be_polarized_by_neighbours++;
                        
                    }             
                
                    mydata->new_source = 0;
                }

				set_motors(0,0);

				set_move_type(STOP);
				set_bot_state(WAIT);
				
                // If is has been orbiting for the entire orbiting block but hasn't changed neighbours, it waits triple the time for the next orbiting block
				if(mydata->counter == (MAX_COUNTER_ORBITING + 1) && mydata->n_different_neighbors < DIFFERENT_NEIGHBORS_TH){
				    mydata->counter = COUNTER_WAIT*3;
			    }
		        else{
    				mydata->counter = 0;
                }
			}

            // ORBITING
			else if(mydata->N_Neighbors > 0){
			
			    // Beginning of a new orbiting block
			    if(mydata->counter == (MAX_COUNTER_ORBITING + 1)){
			    
			        reset_initial_neighbours();
			        mydata->n_different_neighbors = 0;
                    mydata->counter = 0;
			    
			    }
			

                if(MOTORS){
                
				    uint8_t r1;
				    r1 = rand_byte();
				    uint8_t r2;
				    r2 = rand_byte();

                    // If too close to the closest robot, it vibrates to try to escape
				    if(get_dist_by_id(find_nearest_N_id(2)) <= DIST_CRIT - 6
					    && r1 == 125
					    && r2 > R2
					    ){

                        
					    set_motors(125,125);
					    delay(150);

				    }	

				    uint16_t id = find_nearest_N_id(1);
				    orbit2(get_dist_by_id(id), DIST_CRIT);
				    
				}
				
                if(mydata->counter < MAX_COUNTER_ORBITING){
                    mydata->counter++;
                    
                }
                
                // If it reaches the end of an orbiting block, it counts the number of different neighbours since the beginning of the block
                if(mydata->counter == MAX_COUNTER_ORBITING){
                    
                    mydata->n_different_neighbors = count_n_different_neighbors();
                    mydata->counter++;
                    

                    // If enough different neighbours, it will be eventually allowed to become polarized if it stops near a cluster
                    if(mydata->new_source < 3 && mydata->n_different_neighbors >= DIFFERENT_NEIGHBORS_TH){
                        mydata->new_source = 3;
                    }
                    else if(mydata->new_source >= 3 && mydata->new_source < 256 && mydata->n_different_neighbors >= DIFFERENT_NEIGHBORS_TH)
                        mydata->new_source++;
                    else
                        mydata->new_source = 0;
                }
			}
			
			break;

		case FOLLOW:

			if(follow_to_wait()){

				set_motors(0,0);
				set_move_type(STOP);
				set_bot_state(WAIT);
				return;
			}

			else{

                if(MOTORS){
				    uint16_t id = find_nearest_N_id(1);

                    // It rotates in one direction until it starts being further apart from the nearest robot, then switches direction.
				    move_by_turning(get_diff_dist_by_id(id));
			    }
			}

			break;

		case WAIT:

			if(wait_to_orbit()){

                // If it's polarized and it suddenly moves, it isn't allowed to be polarized and becomes not polarized.
                // It will be allowed if it stops by an edge and become polarized if it stops near polarized robots
                if(polarized_by_neighbours()){

                    mydata->polarized_by_neighbours = 0;
                    mydata->hops_to_morphogen_gradient = 127;
                    mydata->allowed_to_be_polarized_by_neighbours = 2;
                    mydata->new_source = 0;

                }
                else if(polarized_by_patterning()){

                    mydata->polarized_by_patterning = 0;
                    mydata->hops_to_morphogen_gradient = 127;
                    mydata->allowed_to_be_polarized_by_neighbours = 2;
                    mydata->new_source = 0;

                }

				if(MOTORS){
				    spinup_motors();

                    // If the nearest neighbor is too close, it vibrates a bit randomly
				    if(get_dist_by_id(find_nearest_N_id(2)) < DIST_CRIT){
					    uint8_t r = rand_byte();
					    if(r < 127) set_motors(125, 0);
					    else set_motors(0,125);
					    delay(350);

				    }

                    // It starts orbiting clockwise
                    
                    if(mydata->last_turn == LEFT){
                        
				        set_motors(0, kilo_turn_right);
				        set_move_type(RIGHT);
				        
				    }
				    else{
				
				        set_motors(kilo_turn_left, 0);
				        set_move_type(LEFT);
				
				    }
				}
				
				set_bot_state(ORBIT);
				mydata->counter = 0;
				
				reset_initial_neighbours();
			    mydata->n_different_neighbors = 0;
			    
			    mydata->new_source = 0;
			}

			else if(wait_to_follow()){

			    set_bot_state(FOLLOW);
				    
			    if(MOTORS){
				    set_move_type(RIGHT);
				    motion(RIGHT);
				    mydata->move_switch_flag = 1;
			    }
			}
			else{

                // Local gradients algorithm
                wait();
                
                if(!polarized_by_neighbours()){

                    // If at least one neighbough not in WAIT state, it waits
			        if(!check_wait_state()){ 
                        mydata->counter = COUNTER_WAIT;

                    }
			        else if (mydata->counter > 0) 
                        mydata->counter--;
                }
            }

			break;
	}
}


/*
 * In this function:
 *   - Random initialisation of concentration of molecules U and V
 *   - Random ID
 *   - Variables are initialised to default/empty values
 *       - Bot is stopped, in WAIT mode and set to clockwise movement
 *   - The message starts to be sent, with the default/empty variables
 */
void setup() {

    // Initialization of the random generator
    while(get_voltage() == -1);
    rand_seed(rand_hard());
    // rand_seed(kilo_uid+1); //seed the random number generator


    float a0_b;
    float a1_b;

    // Random percentage of concentration of molecules U and V
    a0_b=rand_byte()*100/255.0;
    a1_b=rand_byte()*100/255.0;

    // Number of molecules U and V, from 0 to 6 (continuous values, not discrete)
    mydata->molecules_concentration[0] = (float) a0_b*0.01*6;
    mydata->molecules_concentration[1] = (float) a1_b*0.01*6;

    mydata->sum_diff_concentration = -1;

    // Random ID
    kilo_uid = (rand_byte() << 8) | rand_byte();

    // Lock off (no concurrency problems)
    mydata->message_lock = 0;

    // The bot will start orbiting in the opposite direction, i.e. right (clockwise)
	mydata->last_turn = rand_byte() < 128 ? LEFT : RIGHT;

    // Not moving
    mydata->move_switch_flag = 0;

    // Initialisation
    mydata->N_Neighbors = 0;

    // Not moving
    set_move_type(STOP);

    // In WAIT state
    set_bot_state(PATTERN_FORMATION);

    // Initialization
    mydata->counter = KILO_TICKS_START_COUNTING_DIFF; // kilo_ticks when it'll start counting sum of difference in concentration
    mydata->running_avg_NNs = 0;
    mydata->running_avg_Ns = 0;

    mydata->allowed_to_be_polarized_by_neighbours = 2;
    mydata->polarized_by_patterning = 0;
    mydata->polarized_by_neighbours = 0;

    mydata->hops_to_morphogen_gradient = 127;

    mydata->source_signal_id = 0;

    mydata->new_source = 0;

    // The message is initialized
    setup_message();
}



/*
 * Loop function that the kilobots execute continuously. 
 */
void loop(){

    // Processes messages, and updates neighbours tables and N, NN running averages
	receive_inputs();

    // Allows some time to start with the running averages
	if(kilo_ticks == 250){
		mydata->running_avg_Ns = mydata->N_Neighbors;
		mydata->running_avg_NNs = calc_avg_NNs();
	}


    // Patterning phase
    if(get_bot_state() == PATTERN_FORMATION){

        if(mydata->N_Neighbors > 0){
            regulation_linear_model();

            // Start of the counting window for sum of difference in concentration of molecule u
            if(kilo_ticks > mydata->counter){

                mydata->counter = mydata->counter + KILO_TICKS_COUNTING_WINDOW;
                set_bot_state(PATTERN_CHECKING);
                mydata->sum_diff_concentration = 0.0;
            }
        }
    }

    // Patterning phase and checking whether to transition to exploration phase
    else if(get_bot_state() == PATTERN_CHECKING){

        float diff_u = mydata->molecules_concentration[0];

        regulation_linear_model();

        diff_u = diff_u - mydata->molecules_concentration[0];
        if(diff_u < 0)
            diff_u = diff_u * -1;

        mydata->sum_diff_concentration += diff_u;

        // End of the counting window
        if(kilo_ticks > mydata->counter){

            // Pattern stable internally. Transitioning when neighbours are ready, or when there aren't any more neighbours in patterning
            if(mydata->sum_diff_concentration <= MAX_DIFF_CONCENTRATION || phase_transition(0)){

                set_bot_state(PATTERN_READY);
            }
        
            // Pattern not stable. It starts counting window again
            else{

                mydata->counter = mydata->counter + KILO_TICKS_COUNTING_WINDOW;
                mydata->sum_diff_concentration = 0.0;

            }
        }
    }
    
    // Transits to exploration phase when all neighbours are ready
    else if(get_bot_state() == PATTERN_READY){
    
        if(phase_transition(1)){
    
            mydata->counter = 0;
            set_bot_state(WAIT);

            if(polarized_by_polar_threshold()){

                mydata->polarized_by_patterning = 1;
                mydata->allowed_to_be_polarized_by_neighbours = 1;

            }
            else{

                mydata->polarized_by_patterning = 0;
                mydata->allowed_to_be_polarized_by_neighbours = 2;

            }
        }
    }

    // Exploration phase
    else{
		edge_flow();
	}


    /* GRADIENT COLOURS*/

    // Colour code for patterning phase
    if(get_bot_state() == PATTERN_FORMATION || get_bot_state() == PATTERN_CHECKING || get_bot_state() == PATTERN_READY){

        // Green
        if(mydata->molecules_concentration[0] > POLAR_TH) set_color(RGB(0,3,0));

        // Cyan
	    else if (mydata->molecules_concentration[0] > POLAR_TH - 1 && mydata->molecules_concentration[0] <= POLAR_TH) set_color(RGB(0,3,3));

        // Blue
	    else if (mydata->molecules_concentration[0] > POLAR_TH - 2 && mydata->molecules_concentration[0] <= POLAR_TH - 1) set_color(RGB(0,0,3));

        // Pink
	    else if (mydata->molecules_concentration[0] > POLAR_TH - 3 && mydata->molecules_concentration[0] <= POLAR_TH - 2) set_color(RGB(3,0,3));

        // No colour
	    else set_color(RGB(0,0,0));
    }

    // Colour code for exploration phase
    else{

        // Source of a morphogen gradient
        if(mydata->hops_to_morphogen_gradient == 0) {
            
            if(polarized_by_neighbours()){ 

                if(mydata->new_source == 0) 

                    // Blue
                    set_color(RGB(0,0,3)); // Source, polarized and NOT new source

                else if(mydata->new_source == 1) 

                    // Purple
                    set_color(RGB(3,0,3)); // Source, polarized and new source

                else if(mydata->new_source == 2) 

                    // Cyan
                    set_color(RGB(0,3,3)); // Waiting for neighbours to have my source_id


            }
            else // Source and not polarized

                // Cyan
                set_color(RGB(0,3,3));
        }

        else if(mydata->new_source == 2){

            // Cyan
            set_color(RGB(0,3,3)); // Waiting for neighbours to have my source_id

        }

        // White
	    else if(get_bot_state() == ORBIT) set_color(RGB(3,3,3));

        // Red 
	    else if(get_bot_state() == FOLLOW) set_color(RGB(3,0,0));

        // Green
        else if(polarized_by_neighbours()) set_color(RGB(0,3,0));

        // Yellow
        else if(polarized_by_patterning()) set_color(RGB(3,3,0));

        // No colour
	    else set_color(RGB(0,0,0));

    }


    // A unique, local ID is eventually found for the robot
	if(has_neighbour_with_id(kilo_uid)) {
		kilo_uid = reset_id();
		set_color(RGB(3,3,0));
	}

    // Old entries are removed from neighbours table
	purgeNeighbors();

    // After patterning, robots don't change their message unless they're signaling robots or polarized
    if(get_bot_state() == PATTERN_FORMATION || get_bot_state() == PATTERN_CHECKING || get_bot_state() == PATTERN_READY)
    	setup_message();
    else
        setup_message_morphogen_gradient();
}

extern char* (*callback_botinfo) (void);
int16_t circle_barrier(double x, double y, double * dx, double * dy);
char *botinfo(void);


int main(void)
{
  kilo_init();

#ifdef DEBUG
  debug_init();
#endif

#ifdef SIMULATOR
  SET_CALLBACK(botinfo, botinfo);
  SET_CALLBACK(reset, setup);
 #endif

#ifndef KILOBOT
  callback_botinfo = botinfo;
#endif


  //initialize ring buffer
  RB_init();
  kilo_message_rx = rxbuffer_push;
  kilo_message_tx = message_tx;   // register our transmission function

  kilo_start(setup, loop);

  return 0;
}

#ifndef KILOBOT
static char botinfo_buffer[10000];
// provide a text string for the status bar, about this bot
char *botinfo(void)
{
	int n;
	  char *p = botinfo_buffer;
	  n = sprintf (p, "ID: %d  ", kilo_uid);
	  p += n;


	 //SPOTS
	 
	 
	   n = sprintf (p, "N: %i ", mydata->N_Neighbors);
	   p += n;
	   
	   n = sprintf (p, "avg_Ns: %f ", mydata->running_avg_Ns);
	   p += n;
	   
	   n = sprintf (p, "avg_NNs: %f ", mydata->running_avg_NNs);
	   p += n;
	   
	   switch(get_bot_state()){
	   
	    case WAIT:
	    
	        n = sprintf (p, "state: WAIT ");
	        p += n;
	    
	        break;
	        
	    case ORBIT:
	    
	        n = sprintf (p, "state: ORBIT ");
	        p += n;
	    
	        break;
	        
        case FOLLOW:
        
            n = sprintf (p, "state: FOLLOW ");
	        p += n;
	    
	        break;
	        
        case PATTERN_FORMATION:
        
            n = sprintf (p, "state: PATTERN_FORMATION ");
	        p += n;
	    
	        break;
	        
        case PATTERN_CHECKING:
        
            n = sprintf (p, "state: PATTERN_CHECKING ");
	        p += n;
	    
	        break;
	        
        case PATTERN_READY:
        
            n = sprintf (p, "state: PATTERN_READY ");
	        p += n;
	    
	        break;
	   
	   }
	   
	   n = sprintf (p, "u: %f ", mydata->molecules_concentration[0]);
	   p += n;
	   
	   n = sprintf (p, "v: %f ", mydata->molecules_concentration[1]);
	   p += n;
	   
/*	   n = sprintf (p, "r: %f ", (mydata->running_avg_Ns / mydata->running_avg_NNs));*/
/*	   p += n;*/
/*	   */
/*	   n = sprintf (p, "n_diff: %i ", mydata->n_different_neighbors);*/
/*	   p += n;*/
/*	   */
/*	   n = sprintf (p, "counter: %i ", mydata->counter);*/
/*	   p += n;*/
/*	   */
/*	   n = sprintf (p, "allowed: %i ", mydata->allowed_to_be_polarized_by_neighbours);*/
/*	   p += n;*/
/*	   */
/*	   if(mydata->bot_state == WAIT){*/
/*	       n = sprintf (p, ", 2: %i ", check_wait_state());*/
/*	       p += n;*/
/*	       */
/*	       n = sprintf (p, ", 3: %i ", (!polarized_by_patterning()));*/
/*	       p += n;*/
/*	       */
/*	       n = sprintf (p, ", 4: %i ", mydata->hops_to_morphogen_gradient != 0);*/
/*	       p += n;*/
/*	       */
/*	       n = sprintf (p, ", 5.1: %i ", (!polarized_by_neighbours()));*/
/*	       p += n;*/
/*	       */
/*	       n = sprintf (p, ", 5.2: %i ", (!has_at_least_n_polarized_N(MIN_NUMBER_ROBOTS_POLARIZED_FOR_POLARIZATION)));*/
/*	       p += n;*/
/*	       */
/* 	       n = sprintf (p, ", 6: %i ", (mydata->counter == 0));*/
/*	       p += n;*/
/*	       */
/*	       p += n;*/
/*	   }*/
	   return botinfo_buffer;
}
#endif
