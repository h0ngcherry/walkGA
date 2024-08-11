import random

#init part
def create_gene_for_each_part(action_num):
    def create_a_gene():
        result=[]
        for i in range(0,action_num):
            result.append(random.randint(0,1))
        return result
    r_up_arm=create_a_gene()
    r_down_arm=create_a_gene()
    r_palm=create_a_gene()

    l_up_arm=create_a_gene()
    l_down_arm=create_a_gene()
    l_palm=create_a_gene()

    r_up_leg=create_a_gene()
    r_down_leg=create_a_gene()
    r_foot=create_a_gene()

    l_up_leg=create_a_gene()
    l_down_leg=create_a_gene()
    l_foot=create_a_gene()
    return {"r_up_arm":r_up_arm,"r_down_arm":r_down_arm,"r_palm":r_palm,
            "l_up_arm":l_up_arm,"l_down_arm":l_down_arm,"l_palm":l_palm,
            "r_up_leg":r_up_leg,"r_down_leg":r_down_leg,"r_foot":r_foot,
            "l_up_leg":l_up_leg,"l_down_leg":l_down_leg,"l_foot":l_foot,}

def create_all_gene(robot_num,action_num):
    resultList=[]
    for i in range(0,robot_num):
        resultList.append(create_gene_for_each_part(action_num))
    return resultList

#gene exchange part
def exchange_one_pair_genes(gene1,gene2,action_num,propotion=0.5):
    #take half of each gene
    g1_start_index=random.randint(0,int(action_num*(1-propotion)))
    g1_end_index=g1_start_index+int(action_num*propotion)-1

    new_gene=[]
    new1=gene2[0:g1_start_index]
    new2=gene1[g1_start_index:g1_end_index]
    new3=gene2[g1_end_index:len(gene2)]
    
    return new1+new2+new3

def exchange_all_genes_for_all_robot(gene_pair_list,action_num):
    resultList=[]
    for i in range(0,len(gene_pair_list)):
        cache_dic={}
        robot1=gene_pair_list[i][0]
        robot2=gene_pair_list[i][1]
        k1=robot1.keys()
        for key in k1:
            cache_dic[key]=exchange_one_pair_genes(robot1[key],robot2[key],action_num)
        resultList.append(cache_dic)
    return resultList

#pick part
def pick_two_in_roulette(robot_list,fitness_list,gene_list):
    result=[]
    #create roulette
    fitness_sum=sum(fitness_list)
    roulette_list=[]
    range_start=0
    for i in range(0,len(fitness_list)):
        roulette_list.append((range_start,range_start+fitness_list[i]))
        range_start+=fitness_list[i]+1

    #pick
    for i in range(0,len(robot_list)):
        rand1=random.randint(0,int(fitness_sum))
        rand2=random.randint(0,int(fitness_sum))
        id1=id2=0
        for j in range (0,len(roulette_list)):
            if rand1>=roulette_list[j][0] and rand1<=roulette_list[j][1]:
                id1=j
            if rand2>=roulette_list[j][0] and rand2<=roulette_list[j][1]:
                id2=j
        if id1==id2:
            id2=random.randint(0,len(robot_list)-1)
        # result.append((robot_list[id1],robot_list[id2]))
        result.append((id1,id2))
    result_gene_pair_list=[]
    for i in result:
        # print(i[0])
        # print(i[1])
        # print((gene_list[i[0]],gene_list[i[1]]))
        result_gene_pair_list.append((gene_list[i[0]],gene_list[i[1]]))
    return result_gene_pair_list

#mutate
def mutate_a_gene(gene,action_num,probability=0.1):
    all_key=gene.keys()
    for key in all_key:
        r_can=random.random()
        if r_can<=probability:
            r_index=random.randint(0,action_num-1)
            if gene[key][r_index]==0:
                gene[key][r_index]=1
            else:
                gene[key][r_index]=0
        
def mutate_all(gene_list,action_num,probability=0.05):
    for gene in gene_list:
        mutate_a_gene(gene,action_num,probability)

#large mutate to jump out overfitting
def big_mutate_all(gene_list,action_num,probability=0.8):
    for gene in gene_list:
        mutate_a_gene(gene,action_num)