#! /usr/bin/python3
import os
import collections
import pickle
import random

from modules.corpus.corpus import CorpusElement
from modules.mutation import tools

def get_all_checkpoints(ck_path):
    # ga_checkpoints_path
    only_files = os.listdir(ck_path)

    pre_pop_pool = []

    for i in range(len(only_files)):
        with open(ck_path+'/'+only_files[i], "rb") as f:
            if "generation" not in only_files[i]:
                continue
            try:
                pre_pop = pickle.load(f)
                pre_pop_pool.append(pre_pop)
            except Exception:
                pass

    return pre_pop_pool

def generate_restart_scenarios(runner, ga_logger, global_iter, ck_path, scenario_num, bounds):
	
	pre_pop_pool = get_all_checkpoints(ck_path)			 
	
	new_pop_candidate = []
	new_scenario_list = []
	pop_size = len(pre_pop_pool[0])
	npc_size = len(pre_pop_pool[0][0].scenario[1])
	# time_size = len(pre_pop_pool[0][0].scenario[0])
	scenario_size = len(pre_pop_pool[0])
	pop_pool_size = len(pre_pop_pool)
	scenario_dict = {}

	for i in range(scenario_num):
		# 1. init scenario data (data)
		scenario_data = [[], [], []]
		ego_start_pos = random.randrange(bounds[0][0][0], bounds[0][0][1])
		ego_end_pos = random.randrange(bounds[0][1][0], bounds[0][1][1])
		scenario_data[0].append(ego_start_pos)
		scenario_data[0].append(ego_end_pos)
		
		# set npc at per lane
		range_start = bounds[1][1][0]
		range_end = bounds[1][1][1]
		interval = (range_end - range_start) / npc_size
		
		for npc_index in range(npc_size):
			
			segment_start = range_start + npc_index * interval
			segment_end = segment_start + interval
			
			forward_num = random.randint(int(segment_start), int(segment_end) - 1)
			npc_pos = [npc_index, forward_num]
			scenario_data[1].append(npc_pos)
		values = [scenario_data[1][i][1] for i in range(npc_size)]
		random.shuffle(values)
		for i in range(npc_size):
			scenario_data[1][i][1] = values[i]
			
		if random.random() <0.5:
			rain_rate = 0 
			fog_rate = 0 
			wetness_rate = 0
			cloudiness_rate = 0 
		else:
			rain_rate = 0 if random.random()<0.5 else random.uniform(bounds[2][0][0], bounds[2][0][1])
			fog_rate = 0 if random.random()<0.5 else random.uniform(bounds[2][0][0], bounds[2][0][1])
			wetness_rate = 0 if random.random()<0.5 else random.uniform(bounds[2][0][0], bounds[2][0][1])
			cloudiness_rate = 0 if random.random()<0.5 else random.uniform(bounds[2][0][0], bounds[2][0][1])
		scenario_data[2].append(rain_rate)
		scenario_data[2].append(fog_rate)
		scenario_data[2].append(wetness_rate)
		scenario_data[2].append(cloudiness_rate)
	
		daytime = random.randint(bounds[2][1][0], bounds[2][1][1])
		scenario_data[2].append(daytime)

		new_pop_candidate.append(scenario_data)

	# Go through every scenario

	for i in range(scenario_num):
		similarity = 0
		for j in range(pop_pool_size):
			simi_pop = 0
			for k in range(scenario_size):
				# TODO
				scenario1 = new_pop_candidate[i]
				scenario2 = pre_pop_pool[j][k].scenario
				simi = tools.get_similarity_between_scenarios(scenario1, scenario2)
				simi_pop += simi

			simi_pop /= scenario_size + 0.0
			similarity += simi_pop
		similarity /= pop_pool_size + 0.0
		scenario_dict[i] = similarity

	sorted_x = sorted(scenario_dict.items(), key=lambda kv: kv[1], reverse=True)
	sorted_dict = collections.OrderedDict(sorted_x)

	index = sorted_dict.keys()

	j = 0

	for i in index:
		if j == pop_size:
			break
		# run pop
		fitness, scenario_id = runner.run(new_pop_candidate[i])

		new_element = CorpusElement(scenario_id, new_pop_candidate[i], fitness)

		new_scenario_list.append(new_element)
		
		with open(ga_logger, 'a') as f:
			f.write('global_' + str(global_iter) + '_restart_' + str(j))
			f.write(',')
			f.write(scenario_id)
			f.write('\n')
		
		j += 1

	return new_scenario_list

def generate_restart_scenarios_int(runner, ga_logger, global_iter, ck_path, scenario_num, bounds):
    pre_pop_pool = get_all_checkpoints(ck_path)
    new_pop_candidate = []
    new_scenario_list = []
    pop_size = len(pre_pop_pool[0])
    npc_size = len(pre_pop_pool[0][0].scenario[1])
    scenario_size = len(pre_pop_pool[0])
    pop_pool_size = len(pre_pop_pool)
    scenario_dict = {}
    
    for i in range(scenario_num):
        scenario_data = [[], [], [], []]
        ego_start_pos = random.randrange(bounds[0][0][0], bounds[0][0][1])
        if ego_start_pos == 0:
            ego_end_pos = random.randrange(bounds[0][1][0], bounds[0][1][1])
        elif ego_start_pos == 1:
            ego_end_pos = random.randrange(bounds[0][1][0], bounds[0][1][1])    
        scenario_data[0].append(ego_start_pos)
        scenario_data[0].append(ego_end_pos)
        
        npc_start_values = random.sample(range(bounds[1][0][0], bounds[1][0][1]), npc_size)
        for npc_index in range(npc_size):
            npc_start = npc_start_values[npc_index]
            npc_forward = random.randrange(bounds[1][1][0], bounds[1][1][1])
            npc_speed = random.randrange(bounds[1][2][0], bounds[1][2][1])
            npc_pos = [npc_start, npc_forward, npc_speed]
            scenario_data[1].append(npc_pos)
        
        if random.random() <0.5:
            rain_rate = 0 
            fog_rate = 0 
            wetness_rate = 0
            cloudiness_rate = 0 
        else:
            rain_rate = 0 if random.random()<0.5 else random.uniform(bounds[2][0][0], bounds[2][0][1])
            fog_rate = 0 if random.random()<0.5 else random.uniform(bounds[2][0][0], bounds[2][0][1])
            wetness_rate = 0 if random.random()<0.5 else random.uniform(bounds[2][0][0], bounds[2][0][1])
            cloudiness_rate = 0 if random.random()<0.5 else random.uniform(bounds[2][0][0], bounds[2][0][1])
        scenario_data[2].append(rain_rate)
        scenario_data[2].append(fog_rate)
        scenario_data[2].append(wetness_rate)
        scenario_data[2].append(cloudiness_rate)
        daytime = random.randint(bounds[2][1][0], bounds[2][1][1])
        scenario_data[2].append(daytime)
        
        signal = random.randint(bounds[3][0], bounds[3][1])
        scenario_data[3].append(signal)
        
        new_pop_candidate.append(scenario_data)
    
    for i in range(scenario_num):
        similarity = 0
        for j in range(pop_pool_size):
            simi_pop = 0
            for k in range(scenario_size):
				# TODO
                scenario1 = new_pop_candidate[i]
                scenario2 = pre_pop_pool[j][k].scenario
                simi = tools.get_similarity_between_scenarios(scenario1, scenario2)
                simi_pop += simi
            simi_pop /= scenario_size + 0.0
        similarity += simi_pop
        similarity /= pop_pool_size + 0.0
        scenario_dict[i] = similarity
    sorted_x = sorted(scenario_dict.items(), key=lambda kv: kv[1], reverse=True)
    sorted_dict = collections.OrderedDict(sorted_x)
    
    index = sorted_dict.keys()
    
    j = 0
    for i in index:
        if j == pop_size:
            break
        fitness, scenario_id = runner.run(new_pop_candidate[i])
        new_element = CorpusElement(scenario_id, new_pop_candidate[i], fitness)
        new_scenario_list.append(new_element)
        with open(ga_logger, 'a') as f:
            f.write('global_' + str(global_iter) + '_restart_' + str(j))
            f.write(',')
            f.write(scenario_id)
            f.write('\n')
        j += 1
    return new_scenario_list
    

def get_similarity_scenario_vs_pre_pop(scenario, pre_pop_pool):
	
	similarity = 0
	for i in pre_pop_pool:
		pop_similarity = 0
		for j in i:
			simi = tools.get_similarity_between_scenarios(j.scenario, scenario.scenario)
			pop_similarity += simi
		pop_similarity /= len(i)
		similarity += pop_similarity + 0.0
	similarity /= len(pre_pop_pool) + 0.0

	return similarity	