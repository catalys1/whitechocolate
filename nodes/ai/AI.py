import numpy as np
import rospy

field_width = 3.53

class AI(object):

    def __init__(self, team_side, ally_number):
        super(AI, self).__init__()

        # Which team side (home/away) am I on?
        self.team_side = team_side

        # Am I ally1?
        self.ally1 = (ally_number == 1)

        self.skill_sm = None

        self.start_pos_offense = np.array([-0.40, 0, 0])
        self.start_pos_defense = np.array([-1.25, 0, 0])


    def playOffense(self, position, ball):
        return ball


    def playDefense(self, position, ball):
        pass


    def stop(self, position):
        return position


    def reset_offense(self):
        return self.start_pose_offense


    def reset_offense(self):
        return self.start_pose_defense
        

    # def strategize(self, me, ally, opp1, opp2, ball, game_state):
    #     self.turn = True
    #     if self.ally1:
    #         # rush ball
    #         if self.closeToBall(me, ball) and (self.closeToBall(opp1,ball) or self.closeToBall(opp2,ball) and self.turn):
    #             theta = 90
    #             cmds = [me.x,me.y,theta]
    #             self.turn = False
    #         else:
    #             cmds = self.rush_goal(me, ball)

    #     else:
    #         # be a goalie (i.e., follow line on ball)
    #         # opp = self.calc_opponent_pos(ally, opp1, opp2, ball)
    #         # if opp:
    #         #    cmds = self.rush_goal(me, ally)
    #         # else:
    #         cmds = self.follow_ball_on_line(ball, -1.25)
        
    #     if (self.team_side == 'away') ^ game_state.second_half:
    #         cmds = (cmds[0], cmds[1], cmds[2])
    #     return cmds


    # def follow_ball_on_line(self, ball, x_c):
    #     y_c = ball.y
    #     theta_c = 0
    #     return (x_c, y_c, theta_c)



    # #TODO the following 
    # def spin_90(self, theta):
    #     self.skill_sm = Rotate90StateMachine()
    #     self.skill_sm.start(theta)
        

    # def forward_12inches():
    #     return (me.x, me.y, me.theta)


    # def go_to_center(self, x_c, y_c):
    #     return 

    # def box_formation():
    #     return


    # def update_sm(self, args):
    #     self.skill_sm.update(args)



    # def rush_goal(self, me, ball):
    #     # Use numpy to create vectors
    #     theta = 0
    #     if(me.theta != 90):
    #         theta = me.theta
    #     else:
    #         theta = 0


    #     ballvec = np.array([[ball.x], [ball.y]])
    #     mevec = np.array([[me.x], [me.y]])
    #     goalvec = np.array([[field_width/2], [0]])

    #     # unit vector from ball to goal
    #     uv = goalvec - ballvec
    #     uv = uv/np.linalg.norm(uv)

    #     # compute a position 20cm behind ball, but aligned with goal
    #     p = ballvec - 0.20*uv

    #     # If I am sufficiently close to the point behind the ball,
    #     # or in other words, once I am 21cm behind the ball, just
    #     # drive to the goal.
    #     if np.linalg.norm(p - mevec) < 0.21:
    #         cmdvec = goalvec
    #     else:
    #         cmdvec = p

    #     return (cmdvec.flatten()[0], cmdvec.flatten()[1], theta)



    # def calc_opponent_pos(self, ally, opp1, opp2, ball):
    #     d = 0.2
    #     if abs(ally.x - ball.x) <= d:
    #         if abs(ally.x-opp1.x) <= d:
    #             return 1
    #         if abs(ally.x-opp2.x) <= d:
    #             return 2
    #     return 0


    # def closeToBall(self, player, ball):
    #     d = 0.12 # distance from center of robot to center of ball
    #     if abs((player.x-ball.x)**2 + (player.y-ball.y)**2) <= d**2:
    #         return True
