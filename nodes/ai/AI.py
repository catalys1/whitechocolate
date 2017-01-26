import numpy as np

field_width = 3.53

class AI(object):
    def __init__(self, team_side, ally_number):
        super(AI, self).__init__()

        # Which team side (home/away) am I on?
        self.team_side = team_side

        # Am I ally1?
        self.ally1 = (ally_number == 1)
        

    def strategize(self, me, ally, opp1, opp2, ball, game_state):
        
        if self.ally1:
            # rush ball
            cmds = self.rush_goal(me, ball)

        else:
            # be a goalie (i.e., follow line on ball)
            opp = self.calc_opponent_pos(ally, opp1, opp2, ball)
            if opp:
               cmds = self.rush_goal(me, ally)
            else:
                cmds = self.follow_ball_on_line(ball, -1.25)
            
        return cmds


    def follow_ball_on_line(self, ball, x_c):
        y_c = ball.y
        theta_c = 0
        return (x_c, y_c, theta_c)


    def rush_goal(self, me, ball):
        # Use numpy to create vectors
        ballvec = np.array([[ball.x], [ball.y]])
        mevec = np.array([[me.x], [me.y]])
        goalvec = np.array([[field_width/2], [0]])

        # unit vector from ball to goal
        uv = goalvec - ballvec
        uv = uv/np.linalg.norm(uv)

        # compute a position 20cm behind ball, but aligned with goal
        p = ballvec - 0.20*uv

        # If I am sufficiently close to the point behind the ball,
        # or in other words, once I am 21cm behind the ball, just
        # drive to the goal.
        if np.linalg.norm(p - mevec) < 0.21:
            cmdvec = goalvec
        else:
            cmdvec = p

        return (cmdvec.flatten()[0], cmdvec.flatten()[1], 0)



    def calc_opponent_pos(self, ally, opp1, opp2, ball):
        d = 0.2
        if abs(ally.x - ball.x) <= d:
            if abs(ally.x-opp1.x) <= d:
                return 1
            if abs(ally.x-opp2.x) <= d:
                return 2
        return 0
