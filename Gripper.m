classdef Gripper
    % Class to represent a Gripper in the animation
    
    properties
        initialVertices
        h
    end
    
    methods  
        function obj = Gripper(T, l, w, h, color)
            % Construct an instance of a Gripper
            obj.initialVertices = [
                -w/2 w/2 w/2 -w/2 -w/2 -w/2 w/2 w/2;
                -l/2 -l/2 l/2 l/2 l/2 -l/2 -l/2 l/2;
                0 0 0 0 h h h h
                1 1 1 1 1 1 1 1
                ];

            vertices = T * obj.initialVertices;
            
            faces = [2 3 8 7; 1 4 5 6; 1 2 3 4];
            
            obj.h = patch('Vertices', vertices(1:3,:)', 'Faces', faces, 'FaceColor', color);
        end

        function obj = update(obj, T)
            % update Gripper graphic representation
            vertices = T*obj.initialVertices;
            obj.h.Vertices = vertices(1:3,:)';
        end
    end
end
