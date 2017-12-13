# Udacity CarND Path Planning with MATLAB
This is an implementation of the Term 3 project "Path Planning" using MATLAB. 

I chose to do this because all of my programming experience is in MATLAB, and I find it much easier to work with personally. __I am NOT used to object-oriented program__. All of my work is functional programming. I apologize if my poor implementation of classes offends you!

(I also like how I can use logical vectors and matrices to make a bunch of decisions simultaneously without using a for loop every two lines)

Feel free to contact me (tkrebs@ryerson.ca) if you have questions or comments.

### Install
- Install MATLAB 
- Install [jebej/MatlabWebSocket] following their instructions
    - I installed the /src/ folder to 'C:/MatlabWebSocket/src/', as indicated in START_SERVER.m
- Run START_SERVER.m to start the server
- Run the Udacity Term 3 simulator and it should connect and control the car
- Modify __fcnPATH_PLANNING__ to make changes to how the car is controlled

### About it
  - Made with MATLAB 2017a but it should work with previous versions. 
       - There may be some issues with scalar addition of matrices in earlier versions?
  - [jebej/MatlabWebSocket] - Used to communicate with the Udacity Term 3 Simulator
  - Every time the server recieves a message containing telemetry from the simulator, it runs __fcnPATH_PLANNING__ in fcnPATH_PLANNING.m to generate the next (x,y) points for the simulator. **This is where the heart of the project is!**
  - Right now it is just a basic implementation of the project, using a lot of the methods outlined in Udacity's walkthrough and Q&A and the stuff in my own C++ project (https://github.com/travis-k/CarND)

#### Good:
- The car stays in its lane __usually__.
- It shouldn't hit the car in front.
- If there is a car in front, it looks to the left and right to determine lane change safety
- If safe, it will change lanes

#### Bad:
- The speed adjustment with a car in front is brutally crude, so if a car less than 30 metres in front, then the jerkiness is bad
- Lane changes don't take into account the speed of the car, or the speed of the cars in the other lane
- It only looks to lane change when the boolean __too_close__ is __true__, which shouldn't be the case necessarily
- Lane changes left or right are chosen at random basically
- The car can get confused if two cars are in front going the same speed, this is the only time I got any warning (out of lane)
- It doesn't return to the center lane when done passing

 [jebej/MatlabWebSocket]: <https://github.com/jebej/MatlabWebSocket>

