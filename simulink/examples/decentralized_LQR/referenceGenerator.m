classdef referenceGenerator < matlab.System & ...
     matlab.system.mixin.Propagates
    % referenceGenerator Generates reference waveform for the lower tanks. Outputs the current reference value, 'ref', \bar{x} and \bar{u}
    
    properties(Nontunable)
        Waveform1 = 'Square'; % Waveform
        Waveform2 = 'Square'; % Waveform
        dT = 1; % Sample time (s)
        T = 30; % Window length
        Period1 = 200; % Period (s)
        Amplitude1 = 10; % Amplitude (cm)
        Bias1 = 25; % Bias (cm)
        Phase1 = 0; % Phase (s)
        Period2 = 300;  % Period (s)
        Amplitude2 = 10; % Amplitude (cm)
        Bias2 = 25; % Bias (cm)
        Phase2 = 0; % Phase (s)
    end

    properties(DiscreteState)
        x_bar;
        u_bar;
        dCount;
    end
    
    properties(Hidden,Constant)
       Waveform1Set = matlab.system.StringSet({'Square','Sin','Sawtooth','Step'}); 
       Waveform2Set = matlab.system.StringSet({'Square','Sin','Sawtooth','Step'}); 
    end
    
    % Pre-computed constants
    properties(Access = private)
        cte;
    end
        
    methods(Access = protected)
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
%             obj.x_bar = zeros(4,obj.T);
%             obj.u_bar = zeros(2,obj.T);       
        end
        
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.cte = decentralizedLQRLoadParameters();
            obj.dCount = 0;
            obj.x_bar = zeros(4,obj.T+1);
            obj.u_bar = zeros(2,obj.T); 
            
            if strcmp(obj.Waveform1,'Square')
                for i = 1:obj.T+1
                    obj.x_bar(1,i) = obj.Bias1+obj.Amplitude1*(2*double(rem((i-1)*obj.dT+obj.Phase1,obj.Period1)<obj.Period1/2)-1);
                end
            elseif strcmp(obj.Waveform1,'Sin')
                for i = 1:obj.T+1
                    obj.x_bar(1,i) = obj.Bias1+obj.Amplitude1*(sin(2*pi*((i-1)*obj.dT+obj.Phase1)/obj.Period1));
                end
            end
            
            if strcmp(obj.Waveform2,'Square')
                for i = 1:obj.T+1
                    obj.x_bar(2,i) = obj.Bias2+obj.Amplitude2*(2*double(rem((i-1)*obj.dT+obj.Phase2,obj.Period2)<obj.Period2/2)-1);
                end
            elseif strcmp(obj.Waveform2,'Sin')
                for i = 1:obj.T+1
                    obj.x_bar(2,i) = obj.Bias2+obj.Amplitude2*sin(2*pi*((i-1)*obj.dT+obj.Phase2)/obj.Period2);
                end
            end
                     
            %% Compute \bar{x} and \bar{u}
            for i = 1:obj.T+1
                obj.x_bar(3:4,i) = obj.cte.alpha*[obj.x_bar(1,i); obj.x_bar(2,i); sqrt(obj.x_bar(1,i)*obj.x_bar(2,i))];
                if i ~= obj.T+1
                    obj.u_bar(:,i) = obj.cte.beta*[sqrt(obj.x_bar(1,i)); sqrt(obj.x_bar(2,i))];
                end
            end
        end

        function [ref,x_bar,u_bar] = stepImpl(obj) 
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            % Output current intant
            ref = obj.x_bar(1:2,1);
            x_bar = obj.x_bar;
            u_bar = obj.u_bar; 
            
            
            % Prepare next
            obj.x_bar = [obj.x_bar(:,2:end) zeros(4,1)];
            obj.u_bar = [obj.u_bar(:,2:end) zeros(2,1)];
            if strcmp(obj.Waveform1,'Square')               
                    obj.x_bar(1,end) = obj.Bias1+obj.Amplitude1*(2*double(rem(obj.dCount*obj.dT+obj.dT+obj.Phase1,obj.Period1)<obj.Period1/2)-1);
            elseif strcmp(obj.Waveform1,'Sin')
                    obj.x_bar(1,end) = obj.Bias1+obj.Amplitude1*sin(2*pi*(obj.dCount*obj.dT+(obj.T+1)*obj.dT+obj.Phase1)/obj.Period1);
            end
            
            if strcmp(obj.Waveform2,'Square')
                    obj.x_bar(2,end) = obj.Bias2+obj.Amplitude2*(2*double(rem(obj.dCount*obj.dT+obj.dT+obj.Phase2,obj.Period2)<obj.Period2/2)-1);
            elseif strcmp(obj.Waveform2,'Sin')
                    obj.x_bar(2,end) = obj.Bias2+obj.Amplitude2*sin(2*pi*(obj.dCount*obj.dT+(obj.T+1)*obj.dT+obj.Phase2)/obj.Period2);
            end
            obj.x_bar(3:4,end) = obj.cte.alpha*[obj.x_bar(1,end); obj.x_bar(2,end); sqrt(obj.x_bar(1,end)*obj.x_bar(2,end))];
            obj.u_bar(:,end) = obj.cte.beta*[sqrt(obj.x_bar(1,end)); sqrt(obj.x_bar(2,end))];
            
            obj.dCount = obj.dCount+1;
            
        end
        
        
        function [sz,dt,cp] = getDiscreteStateSpecificationImpl(obj,name)
            if strcmp(name,'x_bar')
                sz = [4 obj.T+1];
                dt = 'double';
                cp = false;
            elseif strcmp(name,'u_bar')
                sz = [2 obj.T];
                dt = 'double';
                cp = false;
            elseif strcmp(name,'dCount')
                sz = [1 1];
                dt = 'double';
                cp = false;
            else
                error(['Error: Incorrect State Name: ', name.']);
            end
        end
        
        
        function varargout = getOutputDataTypeImpl(obj)
             varargout = cell(1, getNumOutputs(obj));
            for i = 1:getNumOutputs(obj)
                varargout{i} = 'double';
            end
        end
        function varargout = getOutputSizeImpl(obj)
            varargout = cell(1, getNumOutputs(obj));
            varargout{1} = [2 1];
            varargout{2} = [4 obj.T+1];
            varargout{3} = [2 obj.T];
        end
        function varargout = isOutputComplexImpl(obj)
            varargout = cell(1, getNumOutputs(obj));
            for i = 1:getNumOutputs(obj)
                varargout{i} = false;
            end            
        end
        function varargout = isOutputFixedSizeImpl(obj)
            % Get outputs fixed size.
            varargout = cell(1, getNumOutputs(obj));
            for i = 1:getNumOutputs(obj)
                varargout{i} = true;
            end            
        end


    end
    
    methods(Static,Access = protected)
        function groups = getPropertyGroupsImpl
%             tab1Group = matlab.system.display.SectionGroup(...
%                 'Title','Tank 1',...
%                 'PropertyList',{'Waveform1','Period1','StepTime1','Amplitude1'});
%             
%             tab2Group = matlab.system.display.SectionGroup(...
%                 'Title','Tank 2',...
%                 'PropertyList',{'Waveform2','Period2','StepTime2','Amplitude2'});
            tab1Group = matlab.system.display.SectionGroup(...
                'Title','General',...
                'PropertyList',{'dT','T'});
            tab2Group = matlab.system.display.SectionGroup(...
                'Title','Tank 1',...
                'PropertyList',{'Waveform1','Period1','Amplitude1','Bias1','Phase1'});
            tab3Group = matlab.system.display.SectionGroup(...
                'Title','Tank 2',...
                'PropertyList',{'Waveform2','Period2','Amplitude2','Bias2','Phase2'});
           
            groups = [tab1Group, tab2Group, tab3Group]; 
        end
    end 
end


%     methods(Access = protected)
%         
% %         function flag = isInactivePropertyImpl(obj,prop)
% %             flag = strcmp(prop,'StepTime1') && ~strcmp(obj.Waveform1,'Step') || ...
% %                 strcmp(prop,'StepTime2') && ~strcmp(obj.Waveform2,'Step') || ...
% %                 strcmp(prop,'Period1') && strcmp(obj.Waveform1,'Step') ||...
% %                 strcmp(prop,'Period2') && strcmp(obj.Waveform2,'Step');
% %         end
% 
% 
%         
%     end
        