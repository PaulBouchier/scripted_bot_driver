from math import pi, radians, degrees, fmod
from scripted_bot_driver.anglr import  Angle

"""
AngleHunter is useful in finding the (nominally declining) error between an initial target
angle and subsequent current angle readings. Create AngleHunter with the target angle or 
heading in radians, and provide a current angle with the update() method to find the 
current error.

Optionally disable the default shortest_path calculation, allowing for tracking multiple 
turns. 

The update() function must be called frequently enough that it doesn't miss a 
current_heading crossing of the 0-2pi boundary. current_heading is expected to be normalized.

The return value indicates the magnitude of the error and the sign indicates the direction 
to reduce the error. 
"""
__author__ = "karim  virani (pondersome)"
__version__ = "1.0"
__license__ = "MIT"

TAU = 2*pi

class AngleHunter:
    def __init__(self, target_angle, initial_heading, shortest_path=True):
        
        self.shortest_path = shortest_path
        self.initial_heading = Angle(initial_heading)

        # don't think of target_angle as a heading, it can be > 2pi magnitude
        self.target_angle = Angle(target_angle) #this is the relative angle requested until the first update()
        if shortest_path:
            self.target_angle += self.initial_heading #target relative to initial heading
            self.target_angle = self.initial_heading.angle_to(self.target_angle.normalized())

        self.last_heading = None # signifies first update
        self.cumulative_angle = Angle(0) # this is also not a heading, but a running count of the angle turned since the first update
        self.error = self.target_angle 
        self.delta_heading = Angle(0)
        self.update(initial_heading)

    def update(self, current_heading):
        self.current_heading = Angle(current_heading).normalized() # allows us to pass non-normalized angles for unit testing even though an odemetry reading should already be normalized
        if self.last_heading is not None:
            self.delta_heading = self.last_heading.angle_to(self.current_heading) # assumes update is called frequently so that a half rotation won't be missed
            self.cumulative_angle += self.delta_heading

        self.last_heading = self.current_heading
        
        self.error = self.target_angle - self.cumulative_angle
        # the sign of the error should indicate the direction of turn that will close the error
        return self.error
    
    def __str__(self):
        return f"AngleHunter: {self.target_angle.radians:.6f} target radians, degrees={self.target_angle.degrees:.6f}, shortest_path={self.shortest_path}))"

    def __repr__(self):   
        repr = f"AngleHunter: shortest_path={self.shortest_path}\n{self.target_angle.radians:.6f} target radians, degrees={self.target_angle.degrees:.6f}\n" 
        if self.last_heading is not None: #no updates yet so can't return any calcs
            repr += f"{self.initial_heading.radians:.6f} inith  radians, degrees={self.initial_heading.degrees:.6f}\n" + \
            f"{self.current_heading.radians:.6f} headn  radians, degrees={self.current_heading.degrees:.6f}\n" + \
            f"{self.delta_heading.radians:.6f} delta  radians, degrees={self.delta_heading.degrees:.6f}\n" + \
            f"{self.cumulative_angle.radians:.6f} cumul  radians, degrees={self.cumulative_angle.degrees:.6f}\n" + \
            f"{self.error.radians:.6f} error  radians, degrees={self.error.degrees:.6f}\n"
        return repr 

    def get_target_radians(self):
        return self.target_angle.radians

    def get_target_degrees(self):
        return self.target_angle.degrees

    def get_cumul_radians(self):
        return self.cumulative_angle.radians

    def get_cumul_degrees(self):
        return self.cumulative_angle.degrees

def d(rad):
    return degrees(rad)

def r(deg):
    return radians(deg)

# Test the AngleHunter class
def test_angle_hunter():
    test_cases = [
        #{'target_angle': 0, 'shortest_path': True, 'headings': [181, 135, 100, 90, 45, 1, 355], 'description':'simple 180 turn approaching and overshooting. errors should decline through 0'},
        #{'target_angle': 0, 'shortest_path': True, 'headings': [180, 235, 270, 330, 345, 359, 365], 'description':'simple 180 turn approaching and overshooting. errors should decline through 0'},
        #{'target_angle': -90, 'shortest_path': True, 'headings': [80, 70, 50, 20, 355, 350],'description':'Approaching -90 degrees with shortest path'},
        #{'target_angle': -180, 'shortest_path': True, 'headings': [0, -90, -180, -270, -360],'description':'Approaching -180 degrees with shortest path'},
        #ok{'target_angle': 540, 'shortest_path': False, 'headings': [0, -45, 90, 180, 270, 360, 450, 540],'description':'Completing 1.5 turns to 540 degrees'},        
        #ok{'target_angle': -360, 'shortest_path': False, 'headings': [0, 45, -90, -180, -270, -360, -450],'description':'Completing -1 turn to -360 degrees'},
        {'target_angle': -355, 'shortest_path': False, 'headings': [355, 340, 300, 220, 180, 90, 0],'description':'Completing -1 turn to -360 degrees'},
        #{'target_angle': 90, 'shortest_path': True, 'headings': [0, 10, 20, 350, 340, 330], 'description':'Testing wrap-around from just below 2pi to just above 0'}
    ]

    for case in test_cases:
        ah = AngleHunter(r(case['target_angle']), case['shortest_path'])
        print(f"\nTesting: {case['description']}")
        print(f"Target angle: {case['target_angle']} degrees ({r(case['target_angle']):.2f} radians), Shortest Path: {case['shortest_path']}")
        for heading in case['headings']:
            error = ah.update(r(heading))
            #print(f"Current Heading: {heading} degrees ({r(heading):.2f} radians), Error: {d(error):.2f} degrees ({error:.2f} radians)")
            print(repr(ah))

        print(f"Last Cumulative Angle: {ah.get_cumul_radians():.2f} radians ({ah.get_cumul_degrees():.2f} degrees)")
        print(f"Target Angle: {ah.get_target_radians():.2f} radians ({ah.get_target_degrees():.2f} degrees)")

if __name__ == '__main__':
    test_angle_hunter()
