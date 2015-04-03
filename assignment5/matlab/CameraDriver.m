%Drive a vehicle based on camera observations

classdef CameraDriver < handle
    properties
        veh
        log
        nrLines
        counter
        prevFilename
    end
    
    methods
        
        function driver = CameraDriver(filename)
            % Constructor
            fileID = fopen(filename);
            driver.log = textscan(fileID, '%d %d %s');
            
            driver.prevFilename = '';
            
            driver.nrLines=size(driver.log{1},1)
            fprintf(1,'read %d lines from %s\n',driver.nrLines,filename);
            driver.counter=1;
        end
        
        function init(driver)
        end
        
        function [dx] = demand(driver)
            dx = [0 0 0]';
            
            % Return dx
            driver.counter = driver.counter + 1;
            if driver.counter>driver.nrLines
                return;
            else
%                 % Motor values given to the Fluke [-200,200]
%                 % Left and right with respect to the Fluke
%                 v = [
%                     driver.log{1}(driver.counter)
%                     driver.log{2}(driver.counter)
%                     ];
%                                 
%                 % get motor values (Left-Right wrpt Fluke)
%                 uL=v(1);
%                 uR=v(2);
%                 
%                 scale = 40.0/2400;  %Change it to accordingly to what you used in Assignment 3
%                 ScaledUL = double(uL)*scale;
%                 ScaledUR = double(uR)*scale;
%                 
%                 r = .01;
%                 L = .014;
%                 
%                 % calculate speed
%                 u(1) =  r*((ScaledUL+ScaledUR)/2.0)*10;
%                 u(2) = (r/L)*(ScaledUR-ScaledUL)*1.0;


                while 1
                    if driver.counter>driver.nrLines
                        return;
                    end
                    cameraFile = driver.log{3}(driver.counter)
                    if cameraFile{1} == '0'
                        if driver.counter> driver.nrLines
                            break;
                        else
                            driver.counter = driver.counter + 1;
                        end
                    else
                        cameraFile = char(cameraFile);
                        
                        %% TODO: do camera things and calculate dx
                        % set previous image features
                        
                        if length(driver.prevFilename) ~= 0
                            [R,t] = imgproc(driver.prevFilename, cameraFile);
                            angles = tr2eul(R)
                            dx = [t; angles(1)];
                        else
                            dx = [0 0 0]'; 
                        end
                        driver.prevFilename = cameraFile;
                     
                        
                        break;
                    end
                end
                
                
            end
        end
        
    end % methods
end % classdef
