%% Function to create a robot model based on DH parameters
function myrobot = mykuka(DH)

    % Initialize an array of Link objects
    links = [Link];   

    % Get the size of the DH parameter matrix
    DHsize = size(DH);

    % Loop through each row of the DH parameter matrix
    for i = 1:DHsize(1)
         links(i) = Link('d', DH(i,2), 'a', DH(i,3), 'alpha', DH(i,4));
    end
   
    % Create a SerialLink robot from the array of Link objects
    myrobot = SerialLink(links, 'name', 'kuka');
    
end
