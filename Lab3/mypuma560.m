
function myrobot = mypuma560(DH)
    % Initialize an array of Link objects
    links = [Link];   
    
    % Determine the number of rows in the DH parameter matrix
    DHsize = size(DH);
    
    % Loop through each row of the DH matrix to create each link of the robot
    for i = 1:DHsize(1)
        % Create a Link object using DH parameters for each robot joint
        links(i) = Link('d',DH(i,2), 'a', DH(i,3), 'alpha', DH(i,4));
    end
   
    % Create the robot model
    myrobot = SerialLink(links,'name', 'puma560');
    
end
