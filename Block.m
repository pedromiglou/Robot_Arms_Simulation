classdef Block
    % Class to represent a Block in the animation
    
    properties
        initialVertices
        h
    end
    
    methods  
        function obj = Block(T, LBL, WBL, HBL, color)
            % Construct an instance of a Block
            obj.initialVertices = [
                -WBL/2 WBL/2 WBL/2 -WBL/2 -WBL/2 -WBL/2 WBL/2 WBL/2;
                -LBL/2 -LBL/2 LBL/2 LBL/2 LBL/2 -LBL/2 -LBL/2 LBL/2;
                0 0 0 0 HBL HBL HBL HBL
                1 1 1 1 1 1 1 1
                ];

            vertices = T * obj.initialVertices;
            
            faces = [1 2 3 4; 1 2 7 6; 2 3 8 7; 3 4 5 8; 1 4 5 6; 6 7 8 5];
            
            obj.h = patch('Vertices', vertices(1:3,:)', 'Faces', faces, 'FaceColor', color);
        end

        function obj = update(obj, T)
            % update Block graphic representation
            vertices = T*obj.initialVertices;
            obj.h.Vertices = vertices(1:3,:)';
        end

        function obj = disappear(obj)
            %make Block disappear
            obj.h.FaceAlpha=0;
        end
    end
end
