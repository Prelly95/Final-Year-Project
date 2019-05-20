% Setup and open the serial connection
s = serial('COM1','BaudRate',115200);   % TODO: Modify port and rate as needed
fopen(s);
try
    % Set a variable
    fprintf(s,'x=123.456\n');

    % Retrieve a variable
    x = query(s,'x?','%s\n','x is %f\n')
    % Note: If you need to troubleshoot the query command, it is equivalent to:
%       fprintf(s,'x?');        % serial fprintf already includes newline
%       x = fscanf(s,'x is %f\n')
catch me
    fclose(s); delete(s); clear s   % Close serial connection and clean up
    rethrow me;                     % Pass to higher level error handler
end
fclose(s); delete(s); clear s       % Close serial connection and clean up

% Note: If the serial object is not cleaned up before opening another connection, 
%       Matlab will lose its handle to the previous serial object and lock the 
%       port, requiring a session restart to release the resource.