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
import math

class Robot:
    def __init__(self,nameId,allSizeDic,space,screen):
        self.nameId=nameId
        self.distance=0
        moment=50
        friction=1
        elasticity=0.1
        max_force=45000000*3
        init_height=(allSizeDic["r_foot"][1]+allSizeDic["r_down_leg"][1]+allSizeDic["r_up_leg"][1]+allSizeDic["trunk"][1]/2)+100
        # init_height=400
        init_trunk_pos=(350,init_height)
        trunk_size=allSizeDic["trunk"]
        
        #create body part except head
        def create_body_part(partName,offset):
            size=allSizeDic[partName]
            shape=pymunk.Poly.create_box(None,size)
            body_moment=pymunk.moment_for_poly(moment,shape.get_vertices())
            body=pymunk.Body(moment,body_moment)
            body.position=(init_trunk_pos[0]+offset[0],init_trunk_pos[1]+offset[1])
            shape.body=body
            shape.friction=friction
            shape.elasticity=elasticity
            shape.color=create_new_color()
            space.add(body,shape)
            return body,shape

        def create_head(offset):
            radius=allSizeDic["head"]
            head_moment=pymunk.moment_for_circle(moment,0,radius)
            body=pymunk.Body(moment,head_moment)
            body.position=(init_trunk_pos[0]+offset[0],init_trunk_pos[1]+offset[1])
            shape=pymunk.Circle(body,radius)
            shape.friction=friction
            shape.elasticity=elasticity
            space.add(body,shape)
            shape.color=create_new_color()
            return body,shape
        
        def add_joint(main_body,part_body,offset1,offset2,space):
            joint=pymunk.PivotJoint(main_body, part_body, offset1, offset2)
            joint.max_force=max_force
            space.add(joint)
            return joint

        def add_motor_to_joint(main_body,part_body,rate,space):
            motor = pymunk.SimpleMotor(main_body, part_body, rate)
            motor.max_force=max_force
            space.add(motor)
            return motor

        def create_new_color():
            return (random.randint(50,220),random.randint(50,220),random.randint(50,220),0)

        #create all part
        head_offset=(0,trunk_size[1]/2+allSizeDic["head"])
        r_up_arm_offset=(trunk_size[0]/2+allSizeDic["r_up_arm"][0]/2,trunk_size[1]/2-allSizeDic["r_up_arm"][1]/2)
        l_up_arm_offset=(-trunk_size[0]/2-allSizeDic["l_up_arm"][0]/2,trunk_size[1]/2-allSizeDic["l_up_arm"][1]/2)
        r_down_arm_offset=(r_up_arm_offset[0]+allSizeDic["r_up_arm"][0]/2+allSizeDic["r_down_arm"][0]/2,r_up_arm_offset[1])
        l_down_arm_offset=(l_up_arm_offset[0]-allSizeDic["l_up_arm"][0]/2-allSizeDic["l_down_arm"][0]/2,l_up_arm_offset[1])
        r_palm_offset=(r_down_arm_offset[0]+allSizeDic["r_down_arm"][0]/2+allSizeDic["r_palm"][0]/2,r_down_arm_offset[1])
        l_palm_offset=(l_down_arm_offset[0]-allSizeDic["l_down_arm"][0]/2-allSizeDic["l_palm"][0]/2,l_down_arm_offset[1])
        r_up_leg_offset=(trunk_size[0]/2-allSizeDic["r_up_leg"][0]/2,-trunk_size[1]/2-allSizeDic["r_up_leg"][1]/2)
        l_up_leg_offset=(-trunk_size[0]/2+allSizeDic["l_up_leg"][0]/2,-trunk_size[1]/2-allSizeDic["l_up_leg"][1]/2)
        r_down_leg_offset=(r_up_leg_offset[0],r_up_leg_offset[1]-allSizeDic["r_up_leg"][1]/2-allSizeDic["r_down_leg"][1]/2)
        l_down_leg_offset=(l_up_leg_offset[0],l_up_leg_offset[1]-allSizeDic["l_up_leg"][1]/2-allSizeDic["l_down_leg"][1]/2)
        r_foot_offset=(r_down_leg_offset[0],r_down_leg_offset[1]-allSizeDic["r_down_leg"][1]/2-allSizeDic["r_foot"][1]/2)
        l_foot_offset=(l_down_leg_offset[0],l_down_leg_offset[1]-allSizeDic["l_down_leg"][1]/2-allSizeDic["l_foot"][1]/2)
        self.trunk_body,self.trunk_shape=create_body_part("trunk",(0,0))
        self.head_body,self.head_shape=create_head(head_offset)
        self.r_up_arm_body,self.r_up_arm_shape=create_body_part("r_up_arm",r_up_arm_offset)
        self.l_up_arm_body,self.l_up_arm_shape=create_body_part("l_up_arm",l_up_arm_offset)
        self.r_down_arm_body,self.r_down_arm_shape=create_body_part("r_down_arm",r_down_arm_offset)
        self.l_down_arm_body,self.l_down_arm_shape=create_body_part("l_down_arm",l_down_arm_offset)
        self.r_palm_body,self.r_palm_shape=create_body_part("r_palm",r_palm_offset)
        self.l_palm_body,self.l_palm_shape=create_body_part("l_palm",l_palm_offset)
        self.r_up_leg_body,self.r_up_leg_shape=create_body_part("r_up_leg",r_up_leg_offset)
        self.l_up_leg_body,self.l_up_leg_shape=create_body_part("l_up_leg",l_up_leg_offset)
        self.r_down_leg_body,self.r_down_leg_shape=create_body_part("r_down_leg",r_down_leg_offset)
        self.l_down_leg_body,self.l_down_leg_shape=create_body_part("l_down_leg",l_down_leg_offset)
        self.r_foot_body,self.r_foot_shape=create_body_part("r_foot",r_foot_offset)
        self.l_foot_body,self.l_foot_shape=create_body_part("l_foot",l_foot_offset)

        #add all joints
        ne_off1=(-4,allSizeDic["trunk"][1]/2)
        ne_off2=(-4,-allSizeDic["head"])
        ne2_off1=(4,allSizeDic["trunk"][1]/2)
        ne2_off2=(4,-allSizeDic["head"])
        rs_off1=(allSizeDic["trunk"][0]/2,allSizeDic["trunk"][1]/2-allSizeDic["r_up_arm"][1]/2)
        rs_off2=(-allSizeDic["r_up_arm"][0]/2,0)
        ls_off1=(-allSizeDic["trunk"][0]/2,allSizeDic["trunk"][1]/2-allSizeDic["l_up_arm"][1]/2)
        ls_off2=(allSizeDic["l_up_arm"][0]/2,0)
        re_off1=(allSizeDic["r_up_arm"][0]/2,0)
        re_off2=(-allSizeDic["r_down_arm"][0]/2,0)
        le_off1=(-allSizeDic["l_up_arm"][0]/2,0)
        le_off2=(allSizeDic["l_down_arm"][0]/2,0)
        rw_off1=(allSizeDic["r_down_arm"][0]/2,0)
        rw_off2=(-allSizeDic["r_palm"][0]/2,0)
        lw_off1=(-allSizeDic["l_down_arm"][0]/2,0)
        lw_off2=(allSizeDic["l_palm"][0]/2,0)
        rh_off1=(allSizeDic["trunk"][0]/2-allSizeDic["r_up_leg"][0]/2,-allSizeDic["trunk"][1]/2)
        rh_off2=(0,allSizeDic["r_up_leg"][1]/2)
        lh_off1=(-allSizeDic["trunk"][0]/2+allSizeDic["l_up_leg"][0]/2,-allSizeDic["trunk"][1]/2)
        lh_off2=(0,allSizeDic["l_up_leg"][1]/2)
        rk_off1=(0,-allSizeDic["r_up_leg"][1]/2)
        rk_off2=(0,allSizeDic["r_down_leg"][1]/2)
        lk_off1=(0,-allSizeDic["l_up_leg"][1]/2)
        lk_off2=(0,allSizeDic["l_down_leg"][1]/2)
        ra_off1=(0,-allSizeDic["l_down_leg"][1]/2)
        ra_off2=(0,allSizeDic["r_foot"][1]/2)
        la_off1=(0,-allSizeDic["l_down_leg"][1]/2)
        la_off2=(0,allSizeDic["l_foot"][1]/2)
        
        self.neck_joint=add_joint(self.trunk_body,self.head_body,ne_off1,ne_off2,space)
        self.neck_joint2=add_joint(self.trunk_body,self.head_body,ne2_off1,ne2_off2,space)
        self.r_shoulder_joint=add_joint(self.trunk_body,self.r_up_arm_body,rs_off1,rs_off2,space)
        self.l_shoulder_joint=add_joint(self.trunk_body,self.l_up_arm_body,ls_off1,ls_off2,space)
        self.r_elbow_joint=add_joint(self.r_up_arm_body,self.r_down_arm_body,re_off1,re_off2,space)
        self.l_elbow_joint=add_joint(self.l_up_arm_body,self.l_down_arm_body,le_off1,le_off2,space)
        self.r_wrist_joint=add_joint(self.r_down_arm_body,self.r_palm_body,rw_off1,rw_off2,space)
        self.l_wrist_joint=add_joint(self.l_down_arm_body,self.l_palm_body,lw_off1,lw_off2,space)
        self.r_hip_joint=add_joint(self.trunk_body,self.r_up_leg_body,rh_off1,rh_off2,space)
        self.l_hip_joint=add_joint(self.trunk_body,self.l_up_leg_body,lh_off1,lh_off2,space)
        self.r_knee_joint=add_joint(self.r_up_leg_body,self.r_down_leg_body,rk_off1,rk_off2,space)
        self.l_knee_joint=add_joint(self.l_up_leg_body,self.l_down_leg_body,lk_off1,lk_off2,space)
        self.r_ankle_joint=add_joint(self.r_down_leg_body,self.r_foot_body,ra_off1,ra_off2,space)
        self.l_ankle_joint=add_joint(self.l_down_leg_body,self.l_foot_body,la_off1,la_off2,space)

        #add power to all parts
        # self.neck_motor=add_motor_to_joint(self.trunk_body,self.head_body,0,space)
        self.rs_motor=add_motor_to_joint(self.trunk_body,self.r_up_arm_body,0,space)
        self.re_motor=add_motor_to_joint(self.r_up_arm_body,self.r_down_arm_body,0,space)
        self.rw_motor=add_motor_to_joint(self.r_down_arm_body,self.r_palm_body,0,space)
        self.ls_motor=add_motor_to_joint(self.trunk_body,self.l_up_arm_body,0,space)
        self.le_motor=add_motor_to_joint(self.l_up_arm_body,self.l_down_arm_body,0,space)     
        self.lw_motor=add_motor_to_joint(self.l_down_arm_body,self.l_palm_body,0,space)
        self.rh_motor=add_motor_to_joint(self.trunk_body,self.r_up_leg_body,0,space)
        self.rk_motor=add_motor_to_joint(self.r_up_leg_body,self.r_down_leg_body,0,space)
        self.ra_motor=add_motor_to_joint(self.r_down_leg_body,self.r_foot_body,0,space)
        self.lh_motor=add_motor_to_joint(self.trunk_body,self.l_up_leg_body,0,space)
        self.lk_motor=add_motor_to_joint(self.l_up_leg_body,self.l_down_leg_body,0,space)
        self.la_motor=add_motor_to_joint(self.l_down_leg_body,self.l_foot_body,0,space)
        
        #remove collision
        no_collision = pymunk.ShapeFilter(group=1)
        self.head_shape.filter=no_collision
        self.trunk_shape.filter=no_collision
        self.r_up_arm_shape.filter=no_collision
        self.r_down_arm_shape.filter=no_collision
        self.r_palm_shape.filter=no_collision
        self.l_up_arm_shape.filter=no_collision
        self.l_down_arm_shape.filter=no_collision
        self.l_palm_shape.filter=no_collision
        self.r_up_leg_shape.filter=no_collision
        self.r_down_leg_shape.filter=no_collision
        self.r_foot_shape.filter=no_collision
        self.l_up_leg_shape.filter=no_collision
        self.l_down_leg_shape.filter=no_collision
        self.l_foot_shape.filter=no_collision

        #Rotation angle limitation,true
        self.rs_can_move=True
        self.re_can_move=True
        self.rw_can_move=True
        self.ls_can_move=True
        self.le_can_move=True
        self.lw_can_move=True
        self.rh_can_move=True
        self.rk_can_move=True
        self.ra_can_move=True
        self.lh_can_move=True
        self.lk_can_move=True
        self.la_can_move=True
    
    def increase_distance(self,movement):
        self.distance+=movement

    def detect_limits(self):
        self.rs_can_move=True
        self.re_can_move=True
        self.rw_can_move=True
        self.ls_can_move=True
        self.le_can_move=True
        self.lw_can_move=True
        self.rh_can_move=True
        self.rk_can_move=True
        self.ra_can_move=True
        self.lh_can_move=True
        self.lk_can_move=True
        self.la_can_move=True
        #right arm
        if math.degrees(self.trunk_body.angle)-math.degrees(self.r_up_arm_body.angle)>= 90 and self.rs_motor.rate > 0:
            self.rs_motor.rate = 0
            self.rs_can_move = False
        elif math.degrees(self.trunk_body.angle)-math.degrees(self.r_up_arm_body.angle)<= -90 and self.rs_motor.rate < 0:
            self.rs_motor.rate = 0
            self.rs_can_move = False

        if math.degrees(self.r_up_arm_body.angle)-math.degrees(self.r_down_arm_body.angle)>= 90 and self.re_motor.rate > 0:
            self.re_motor.rate = 0
            self.re_can_move = False
        elif math.degrees(self.r_up_arm_body.angle)-math.degrees(self.r_down_arm_body.angle)<= -90 and self.re_motor.rate < 0:
            self.re_motor.rate = 0
            self.re_can_move = False

        if math.degrees(self.r_down_arm_body.angle)-math.degrees(self.r_palm_body.angle)>= 90 and self.rw_motor.rate > 0:
            self.rw_motor.rate = 0
            self.rw_can_move = False
        elif math.degrees(self.r_down_arm_body.angle)-math.degrees(self.r_palm_body.angle)<= -90 and self.rw_motor.rate < 0:
            self.rw_motor.rate = 0
            self.rw_can_move = False
        
        #left arm
        if math.degrees(self.trunk_body.angle)-math.degrees(self.l_up_arm_body.angle)>= 90 and self.ls_motor.rate > 0:
            self.ls_motor.rate = 0
            self.ls_can_move = False
        elif math.degrees(self.trunk_body.angle)-math.degrees(self.l_up_arm_body.angle)<= -90 and self.ls_motor.rate < 0:
            self.ls_motor.rate = 0
            self.ls_can_move = False

        if math.degrees(self.l_up_arm_body.angle)-math.degrees(self.l_down_arm_body.angle)>= 90 and self.le_motor.rate > 0:
            self.le_motor.rate = 0
            self.le_can_move = False
        elif math.degrees(self.l_up_arm_body.angle)-math.degrees(self.l_down_arm_body.angle)<= -90 and self.le_motor.rate < 0:
            self.le_motor.rate = 0
            self.le_can_move = False

        if math.degrees(self.l_down_arm_body.angle)-math.degrees(self.l_palm_body.angle)>= 90 and self.lw_motor.rate > 0:
            self.lw_motor.rate = 0
            self.lw_can_move = False
        elif math.degrees(self.l_down_arm_body.angle)-math.degrees(self.l_palm_body.angle)<= -90 and self.lw_motor.rate < 0:
            self.lw_motor.rate = 0
            self.lw_can_move = False

        #right leg
        if math.degrees(self.trunk_body.angle)-math.degrees(self.r_up_leg_body.angle)>= 90 and self.rh_motor.rate > 0:
            self.rh_motor.rate = 0
            self.rh_can_move = False
        elif math.degrees(self.trunk_body.angle)-math.degrees(self.r_up_leg_body.angle)<= -90 and self.rh_motor.rate < 0:
            self.rh_motor.rate = 0
            self.rh_can_move = False

        if math.degrees(self.r_up_leg_body.angle)-math.degrees(self.r_down_leg_body.angle)>= 90 and self.rk_motor.rate > 0:
            self.rk_motor.rate = 0
            self.rk_can_move = False
        elif math.degrees(self.r_up_leg_body.angle)-math.degrees(self.r_down_leg_body.angle)<= -90 and self.rk_motor.rate < 0:
            self.rk_motor.rate = 0
            self.rk_can_move = False

        if math.degrees(self.r_down_leg_body.angle)-math.degrees(self.r_foot_body.angle)>= 90 and self.ra_motor.rate > 0:
            self.ra_motor.rate = 0
            self.ra_can_move = False
        elif math.degrees(self.r_down_leg_body.angle)-math.degrees(self.r_foot_body.angle)<= -90 and self.ra_motor.rate < 0:
            self.ra_motor.rate = 0
            self.ra_can_move = False

        #left leg
        if math.degrees(self.trunk_body.angle)-math.degrees(self.l_up_leg_body.angle)>= 90 and self.lh_motor.rate > 0:
            self.lh_motor.rate = 0
            self.lh_can_move = False
        elif math.degrees(self.trunk_body.angle)-math.degrees(self.l_up_leg_body.angle)<= -90 and self.lh_motor.rate < 0:
            self.lh_motor.rate = 0
            self.lh_can_move = False

        if math.degrees(self.l_up_leg_body.angle)-math.degrees(self.l_down_leg_body.angle)>= 90 and self.lk_motor.rate > 0:
            self.lk_motor.rate = 0
            self.lk_can_move = False
        elif math.degrees(self.l_up_leg_body.angle)-math.degrees(self.l_down_leg_body.angle)<= -90 and self.lk_motor.rate < 0:
            self.lk_motor.rate = 0
            self.lk_can_move = False

        if math.degrees(self.l_down_leg_body.angle)-math.degrees(self.l_foot_body.angle)>= 90 and self.la_motor.rate > 0:
            self.la_motor.rate = 0
            self.la_can_move = False
        elif math.degrees(self.l_down_leg_body.angle)-math.degrees(self.l_foot_body.angle)<= -90 and self.la_motor.rate < 0:
            self.la_motor.rate = 0
            self.la_can_move = False
    
    def reset_position(self,x,y):
        self.head_body._set_position((self.head_body.position[0]-x,self.head_body.position[1]))
        self.trunk_body._set_position((self.trunk_body.position[0]-x,self.trunk_body.position[1]))

        self.r_up_arm_body._set_position((self.r_up_arm_body.position[0]-x,self.r_up_arm_body.position[1]))
        self.r_down_arm_body._set_position((self.r_down_arm_body.position[0]-x,self.r_down_arm_body.position[1]))
        self.r_palm_body._set_position((self.r_palm_body.position[0]-x,self.r_palm_body.position[1]))

        self.l_up_arm_body._set_position((self.l_up_arm_body.position[0]-x,self.l_up_arm_body.position[1]))
        self.l_down_arm_body._set_position((self.l_down_arm_body.position[0]-x,self.l_down_arm_body.position[1]))
        self.l_palm_body._set_position((self.l_palm_body.position[0]-x,self.l_palm_body.position[1]))

        self.r_up_leg_body._set_position((self.r_up_leg_body.position[0]-x,self.r_up_leg_body.position[1]))
        self.r_down_leg_body._set_position((self.r_down_leg_body.position[0]-x,self.r_down_leg_body.position[1]))
        self.r_foot_body._set_position((self.r_foot_body.position[0]-x,self.r_foot_body.position[1]))

        self.l_up_leg_body._set_position((self.l_up_leg_body.position[0]-x,self.l_up_leg_body.position[1]))
        self.l_down_leg_body._set_position((self.l_down_leg_body.position[0]-x,self.l_down_leg_body.position[1]))
        self.l_foot_body._set_position((self.l_foot_body.position[0]-x,self.l_foot_body.position[1]))
    
    def remove_all_parts(self,space):
        space.remove(self.head_shape,self.head_body)
        space.remove(self.trunk_shape,self.trunk_body)  

        space.remove(self.r_up_arm_shape,self.r_up_arm_body)
        space.remove(self.r_down_arm_shape,self.r_down_arm_body)
        space.remove(self.r_palm_shape,self.r_palm_body)

        space.remove(self.l_up_arm_shape,self.l_up_arm_body)
        space.remove(self.l_down_arm_shape,self.l_down_arm_body)
        space.remove(self.l_palm_shape,self.l_palm_body)

        space.remove(self.r_up_leg_shape,self.r_up_leg_body)
        space.remove(self.r_down_leg_shape,self.r_down_leg_body)
        space.remove(self.r_foot_shape,self.r_foot_body)

        space.remove(self.l_up_leg_shape,self.l_up_leg_body)
        space.remove(self.l_down_leg_shape,self.l_down_leg_body)
        space.remove(self.l_foot_shape,self.l_foot_body)

        space.remove(self.neck_joint)
        space.remove(self.neck_joint2)

        space.remove(self.r_shoulder_joint)
        space.remove(self.r_elbow_joint)
        space.remove(self.r_wrist_joint)

        space.remove(self.l_shoulder_joint)
        space.remove(self.l_elbow_joint)
        space.remove(self.l_wrist_joint)

        space.remove(self.r_hip_joint)
        space.remove(self.r_knee_joint)
        space.remove(self.r_ankle_joint)

        space.remove(self.l_hip_joint)
        space.remove(self.l_knee_joint)
        space.remove(self.l_ankle_joint)

        space.remove(self.rs_motor)
        space.remove(self.re_motor)
        space.remove(self.rw_motor)

        space.remove(self.ls_motor)
        space.remove(self.le_motor)
        space.remove(self.lw_motor)

        space.remove(self.rh_motor)
        space.remove(self.rk_motor)
        space.remove(self.ra_motor)

        space.remove(self.lh_motor)
        space.remove(self.lk_motor)
        space.remove(self.la_motor)
    def zero_speed(self):
        self.rs_motor.rate=0
        self.re_motor.rate=0
        self.rw_motor.rate=0
        self.ls_motor.rate=0
        self.le_motor.rate=0
        self.lw_motor.rate=0
        self.rh_motor.rate=0
        self.rk_motor.rate=0
        self.ra_motor.rate=0
        self.lh_motor.rate=0
        self.lk_motor.rate=0
        self.la_motor.rate=0
    
def forward_move(current_gene,current_robot,action_counter,joint_speed):
    #right arm
    if current_gene["r_up_arm"][action_counter]==1 and current_robot.rs_can_move==True:
        current_robot.rs_motor.rate=joint_speed
    if current_gene["r_up_arm"][action_counter]==0 and current_robot.rs_can_move==True:
        current_robot.rs_motor.rate=-joint_speed
    if current_gene["r_down_arm"][action_counter]==1 and current_robot.re_can_move==True:
        current_robot.re_motor.rate=joint_speed
    if current_gene["r_down_arm"][action_counter]==0 and current_robot.re_can_move==True:
        current_robot.re_motor.rate=-joint_speed
    if current_gene["r_palm"][action_counter]==1 and current_robot.rw_can_move==True:
        current_robot.rw_motor.rate=joint_speed
    if current_gene["r_palm"][action_counter]==0 and current_robot.rw_can_move==True:
        current_robot.rw_motor.rate=-joint_speed

    #left arm
    if current_gene["l_up_arm"][action_counter]==1 and current_robot.ls_can_move==True:
        current_robot.ls_motor.rate=joint_speed
    if current_gene["l_up_arm"][action_counter]==0 and current_robot.ls_can_move==True:
        current_robot.ls_motor.rate=-joint_speed
    if current_gene["l_down_arm"][action_counter]==1 and current_robot.le_can_move==True:
        current_robot.le_motor.rate=joint_speed
    if current_gene["l_down_arm"][action_counter]==0 and current_robot.le_can_move==True:
        current_robot.le_motor.rate=-joint_speed
    if current_gene["l_palm"][action_counter]==1 and current_robot.lw_can_move==True:
        current_robot.lw_motor.rate=joint_speed
    if current_gene["l_palm"][action_counter]==0 and current_robot.lw_can_move==True:
        current_robot.lw_motor.rate=-joint_speed

    #right leg
    if current_gene["r_up_leg"][action_counter]==1 and current_robot.rh_can_move==True:
        current_robot.rh_motor.rate=joint_speed
    if current_gene["r_up_leg"][action_counter]==0 and current_robot.rh_can_move==True:
        current_robot.rh_motor.rate=-joint_speed
    if current_gene["r_down_leg"][action_counter]==1 and current_robot.rk_can_move==True:
        current_robot.rk_motor.rate=joint_speed
    if current_gene["r_down_leg"][action_counter]==0 and current_robot.rk_can_move==True:
        current_robot.rk_motor.rate=-joint_speed
    if current_gene["r_foot"][action_counter]==1 and current_robot.ra_can_move==True:
        current_robot.ra_motor.rate=joint_speed
    if current_gene["r_foot"][action_counter]==0 and current_robot.ra_can_move==True:
        current_robot.ra_motor.rate=-joint_speed

    #left leg
    if current_gene["l_up_leg"][action_counter]==1 and current_robot.lh_can_move==True:
        current_robot.lh_motor.rate=joint_speed
    if current_gene["l_up_leg"][action_counter]==0 and current_robot.lh_can_move==True:
        current_robot.lh_motor.rate=-joint_speed
    if current_gene["l_down_leg"][action_counter]==1 and current_robot.lk_can_move==True:
        current_robot.lk_motor.rate=joint_speed
    if current_gene["l_down_leg"][action_counter]==0 and current_robot.lk_can_move==True:
        current_robot.lk_motor.rate=-joint_speed
    if current_gene["l_foot"][action_counter]==1 and current_robot.la_can_move==True:
        current_robot.la_motor.rate=joint_speed
    if current_gene["l_foot"][action_counter]==0 and current_robot.la_can_move==True:
        current_robot.la_motor.rate=-joint_speed

def backward_move(current_gene,current_robot,action_counter,joint_speed):
    #right arm
    if current_gene["r_up_arm"][action_counter]==1 and current_robot.rs_can_move==True:
        current_robot.rs_motor.rate=-joint_speed
    if current_gene["r_up_arm"][action_counter]==0 and current_robot.rs_can_move==True:
        current_robot.rs_motor.rate=joint_speed
    if current_gene["r_down_arm"][action_counter]==1 and current_robot.re_can_move==True:
        current_robot.re_motor.rate=-joint_speed
    if current_gene["r_down_arm"][action_counter]==0 and current_robot.re_can_move==True:
        current_robot.re_motor.rate=joint_speed
    if current_gene["r_palm"][action_counter]==1 and current_robot.rw_can_move==True:
        current_robot.rw_motor.rate=-joint_speed
    if current_gene["r_palm"][action_counter]==0 and current_robot.rw_can_move==True:
        current_robot.rw_motor.rate=joint_speed

    #left arm
    if current_gene["l_up_arm"][action_counter]==1 and current_robot.ls_can_move==True:
        current_robot.ls_motor.rate=-joint_speed
    if current_gene["l_up_arm"][action_counter]==0 and current_robot.ls_can_move==True:
        current_robot.ls_motor.rate=joint_speed
    if current_gene["l_down_arm"][action_counter]==1 and current_robot.le_can_move==True:
        current_robot.le_motor.rate=-joint_speed
    if current_gene["l_down_arm"][action_counter]==0 and current_robot.le_can_move==True:
        current_robot.le_motor.rate=joint_speed
    if current_gene["l_palm"][action_counter]==1 and current_robot.lw_can_move==True:
        current_robot.lw_motor.rate=-joint_speed
    if current_gene["l_palm"][action_counter]==0 and current_robot.lw_can_move==True:
        current_robot.lw_motor.rate=joint_speed

    #right leg
    if current_gene["r_up_leg"][action_counter]==1 and current_robot.rh_can_move==True:
        current_robot.rh_motor.rate=-joint_speed
    if current_gene["r_up_leg"][action_counter]==0 and current_robot.rh_can_move==True:
        current_robot.rh_motor.rate=joint_speed
    if current_gene["r_down_leg"][action_counter]==1 and current_robot.rk_can_move==True:
        current_robot.rk_motor.rate=-joint_speed
    if current_gene["r_down_leg"][action_counter]==0 and current_robot.rk_can_move==True:
        current_robot.rk_motor.rate=joint_speed
    if current_gene["r_foot"][action_counter]==1 and current_robot.ra_can_move==True:
        current_robot.ra_motor.rate=-joint_speed
    if current_gene["r_foot"][action_counter]==0 and current_robot.ra_can_move==True:
        current_robot.ra_motor.rate=joint_speed

    #left leg
    if current_gene["l_up_leg"][action_counter]==1 and current_robot.lh_can_move==True:
        current_robot.lh_motor.rate=-joint_speed
    if current_gene["l_up_leg"][action_counter]==0 and current_robot.lh_can_move==True:
        current_robot.lh_motor.rate=joint_speed
    if current_gene["l_down_leg"][action_counter]==1 and current_robot.lk_can_move==True:
        current_robot.lk_motor.rate=-joint_speed
    if current_gene["l_down_leg"][action_counter]==0 and current_robot.lk_can_move==True:
        current_robot.lk_motor.rate=joint_speed
    if current_gene["l_foot"][action_counter]==1 and current_robot.la_can_move==True:
        current_robot.la_motor.rate=-joint_speed
    if current_gene["l_foot"][action_counter]==0 and current_robot.la_can_move==True:
        current_robot.la_motor.rate=joint_speed
            

            