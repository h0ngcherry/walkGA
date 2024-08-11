import pygame,sys
import pygame.freetype
import pymunk
from pymunk import pygame_util
from pymunk import Vec2d
import random
import cmath
import gaForWalking
from pygame.locals import *
from pygame.color import *
import robot
from robot import *
from robot import Robot
import os
from gaForWalking import *
import copy
import time
import json


def createLand(space):
    height=36
    body=pymunk.Body(body_type=pymunk.Body.STATIC)
    body.position=(0,height)
    shape=pymunk.Segment(body,(0,height),(2000,height),30)
    shape.friction=10
    shape.elasticity=0.1
    space.add(body,shape)
    return shape

def main():
    pygame.init()
    bgColor=255,255,255
    size=width,height=1200,600

    space = pymunk.Space() 	
    space.gravity = (0.0, -1000.0)
    local_dir = os.path.dirname(__file__)
    bg_path = os.path.join(local_dir, "bg.png")
    bg_land_path = os.path.join(local_dir, "bg_land.png")
    bg=pygame.image.load(bg_path)
    bg_land=pygame.image.load(bg_land_path)		
    bg_rect=bg.get_rect()
    bg_land_rect=bg_land.get_rect()
    bg_land_rect=bg_land_rect.move(0,bg_land_rect[1]+height-bg_land_rect.height)

    screen=pygame.display.set_mode(size)
    pygame.display.set_caption("walkingGA")
    font = pygame.font.SysFont("Arial", 16)
    font2 = pygame.font.SysFont("Arial", 20)
    font3 = pygame.font.SysFont("Arial", 30)
    max_fps=60
    real_fps=0

    fclock=pygame.time.Clock()

    size_dic={"trunk":(50,80),"head":20,
                "r_up_arm":(40,15),"r_down_arm":(40,15),
                "l_up_arm":(40,15),"l_down_arm":(40,15),
                "r_palm":(10,30),"l_palm":(10,30),
                "r_up_leg":(15,50),"l_up_leg":(15,50),
                "r_down_leg":(15,50),"l_down_leg":(15,50),
                "r_foot":(30,10),"l_foot":(30,10)}
   
    createLand(space)
    #create all robots and their genes
    display_flag=False
    skip_timer=0
    fitness_list=[]
    breaker=False
    forward=True
    forward_switcher=1
    
    add_change=1
    change_threshold=20
    best_score=0
    best_generation=1
    joint_speed=4
    robot_list=[]
    gene_list=[]
    robot_num=30
    action_num=6
    for i in range(0,robot_num):
        robot_list.append(Robot(i,size_dic,space,screen))
    gene_list=create_all_gene(robot_num,action_num)
    champion=robot_list[0]
    champion_gene=gene_list[0]
    global_champion=robot_list[1]
    global_champion_gene=gene_list[1]
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    action_counter=0
    iterations=1
    max_iterations=30
    old_champion_distance=champion.distance

    #check the distance of champion per 5s
    MOVE_CHECK = pygame.USEREVENT +1
    pygame.time.set_timer(MOVE_CHECK,10000)
    #iteration starts
    while True:
        this_iteration_best_score=0
        add_change=1
        action_counter=0
        forward=True
        forward_switcher=1
        skip_timer=0
        breaker_timer=0
        bg_moved_dis=0
        #record dead robots and their gene
        garbage_robot_can=[]
        garbage_gene_can=[]
        action_counter=0
        add_change=1
        while True:
            breaker_timer+=1
            if breaker_timer>=5000:
                breaker=True
            if display_flag==False:
                skip_timer+=1
            else:
                skip_timer=0
            
            if champion.distance>this_iteration_best_score:
                this_iteration_best_score=champion.distance
                breaker_timer=0
            else:
                breaker_timer+=1

            #set speed for all part
            for i in range(0,len(gene_list)):
                current_gene=gene_list[i]
                current_robot=robot_list[i]
                if current_robot.trunk_body.position[0]>champion.trunk_body.position[0]:
                    champion=current_robot
                    champion_gene=current_gene
                
                if forward==True:
                    forward_move(current_gene,current_robot,action_counter,joint_speed)
                       
                else:
                    backward_move(current_gene,current_robot,action_counter,joint_speed)
                    
            #change action per threshold frames
            add_change+=1
            if add_change%change_threshold==0:
                if forward==True:
                    action_counter+=1
                    if action_counter>action_num-1:
                        add_change=1
                        action_counter=0
                        forward_switcher+=1
                else:
                    action_counter-=1
                    if action_counter<0:
                        add_change=1
                        action_counter=action_num-1
                        forward_switcher+=1
                       
            if forward_switcher==2:
                forward=not(forward)
                if forward==True:
                    action_counter=0
                else:
                    action_counter=action_num-1
                forward_switcher=0
                                
            # limit the movement of each joint
            for robot in robot_list:
                robot.detect_limits()
            screen.blit(bg,bg_rect)
            for event in pygame.event.get():
                if event.type==pygame.QUIT:
                    sys.exit()
            
            pos1_list=[]
            for i in robot_list:
                pos1_list.append(i.trunk_body.position[0])

            space.step(1/max_fps)

            #record distance
            for i in range(0,len(robot_list)):
                robot_list[i].increase_distance(robot_list[i].trunk_body.position[0]-pos1_list[i])

            if champion.trunk_body.position[0]>=width/2:
                movement=champion.trunk_body.position[0]-width/2
                # champion.increase_distance(movement)
                for i in robot_list:
                    i.reset_position(movement,0)
                bg_rect=bg_rect.move(-movement,0)
                bg_land_rect=bg_land_rect.move(-movement,0)
                bg_moved_dis-=movement
            if display_flag:
                screen.blit(font3.render("Champion display", 5, THECOLORS["black"]), (width-250,0))
            real_fps=fclock.get_fps()
            screen.blit(font.render("fps: " + str(fclock.get_fps()), 1, THECOLORS["black"]), (0,0))
            if display_flag==False:
                screen.blit(font3.render("Evolving", 5, THECOLORS["black"]), (0,40))
                screen.blit(font2.render("Generations: " + str(iterations), 3, THECOLORS["black"]), (width-150,0))
            screen.blit(font2.render("Global Best Score:"+str(int(best_score))+" from generation" + str(best_generation), 5, THECOLORS["black"]), (width/2-150,0))
            screen.blit(font2.render("Current Best Score:"+str(int(champion.distance)), 5, THECOLORS["black"]), (width/2-150,30))
            space.debug_draw(draw_options)
            screen.blit(bg_land,bg_land_rect)
            pygame.display.update()
            fclock.tick(max_fps)

            #remove garbage robot
            garbage_robot_list=[]
            garbage_gene_list=[]
            for i in range(0,len(robot_list)):
                robot_g=robot_list[i]
                if robot_g.trunk_body.position[0]<0:
                    garbage_robot_list.append(robot_list[i])
                    garbage_gene_list.append(gene_list[i])
            
            for i in garbage_robot_list:
                i.remove_all_parts(space)
                robot_list.remove(i)
            for i in garbage_gene_list:
                gene_list.remove(i)
            #record removed robots
            garbage_robot_can+=garbage_robot_list
            garbage_gene_can+=garbage_gene_list

            #zero speed
            for robot in robot_list:
                robot.zero_speed()

            #record fitness
            if breaker==True or len(robot_list)==0 or skip_timer>=1800 or champion.distance>9000:
                breaker=False
                for i in garbage_robot_can:
                    if i.distance<300:
                        i.distance=300
                    fitness_list.append(i.distance)
                for i in robot_list:
                    if i.distance<300:
                        i.distance=300
                    fitness_list.append(i.distance)
    
                for i in gene_list:
                    garbage_gene_can.append(i)
                break
        
        #ga part
        if display_flag==False:
            for i in robot_list:
                i.remove_all_parts(space)
            all_pair_list=pick_two_in_roulette(garbage_gene_can,fitness_list,garbage_gene_can)
            gene_list=exchange_all_genes_for_all_robot(all_pair_list,action_num)
            gene_list[0]=champion_gene
            mutate_all(gene_list,action_num)
            robot_list=[]
            for i in range(0,robot_num):
                robot_list.append(Robot(i,size_dic,space,screen))
            
            if champion.distance>best_score:
                best_score=champion.distance
                best_generation=iterations
                global_champion_gene=champion_gene
            

            #reset screen and other data
            bg_moved_dis=bg_rect[0]-0
            bg_rect=bg_rect.move(-bg_moved_dis,0)
            bg_land_rect=bg_land_rect.move(-bg_moved_dis,0)
            print("iterations: "+str(iterations))
            fitness_list=[]
            iterations+=1
            if iterations>max_iterations:
                display_flag=True
            gene_list[1]=global_champion_gene
            old_champion_distance=0
        if display_flag:
            for i in robot_list:
                i.remove_all_parts(space)
            robot_list=[]
            champion=Robot(0,size_dic,space,screen)
            robot_list.append(champion)
            gene_list=[]
            gene_list.append(global_champion_gene)
            bg_moved_dis=bg_rect[0]-0
            bg_rect=bg_rect.move(-bg_moved_dis,0)
            bg_land_rect=bg_land_rect.move(-bg_moved_dis,0)

    js = json.dumps(global_champion_gene)
    file = open(os.path.join(local_dir, 'best.txt'), 'w')
    file.writelines(js)
    file.writelines("")
    file.writelines("generation"+str(best_generation)+":"+str(int(best_score)))
    file.close()

main()