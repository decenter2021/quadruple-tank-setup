classdef output_amplitude_dft < matlab.System
    % Computes the DFT of the last 4/f measurments (which corresponds to 4
    % periods). It outputs the three most significant harmonics. If 4
    % periods of measurements are not available the DFT is computed for
    % even time-steps and for the maximum number of measurments. The
    % maximum length corresponds to 7200 measurments, i.e., 2 hours of
    % samples taken @1Hz.

    % Public, tunable properties
    properties        
    end

    properties(DiscreteState)
    end
    % Pre-computed constants
    properties(Access = private)
        W;
        buffer;
        counter;
        init;
        Ao;
        f;
    end
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.init = 0; % Flag first iteration
            obj.counter = 0; % Number of available measurments 
            obj.W = 2*3600; % Number of measurments of 4 periods
            obj.buffer = zeros(obj.W,1); % Buffer of measurements
            obj.Ao = zeros(3,1); % Magnitude of 3 highest harmonics
            obj.f = zeros(3,1); % Frequency of the highest magnitude harmonics
        end

        function [Ao,f] = stepImpl(obj,y)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
           global f_global;
            if obj.init == 0
                obj.init = 1; 
                % Define length corresponding to 4 periods
                obj.W = 4*1/f_global;
            end
            % Increment measurment counter
            if obj.counter < obj.W
                obj.counter = obj.counter+1;
            end
            % Update buffer
            obj.buffer(2:end) = obj.buffer(1:end-1);
            obj.buffer(1) = y;
            % Compute DFT for even time-steps
            if rem(obj.counter,2)== 0
                obj.buffer(2:end) = obj.buffer(1:end-1); % Update buffer
                obj.buffer(1) = y;
                % Compute DFT
                [absX,~,wX] = dft_custom(obj.buffer(1:obj.counter),obj.counter);
                % Find first maximum
                absX_aux = absX;
                [magpks,idxpks] = max(absX_aux);
                obj.f(1) = wX(idxpks); % normalized frequency of peaks
                obj.Ao(1) =  magpks;
                absX_aux(idxpks) = 0;
                % Find second maximum
                [magpks,idxpks] = max(absX_aux);
                obj.f(2) = wX(idxpks); % normalized frequency of peaks
                obj.Ao(2) =  magpks;   
                absX_aux(idxpks) = 0;
                % Find third maximum
                [magpks,idxpks] = max(absX_aux);
                obj.f(3) = wX(idxpks); % normalized frequency of peaks
                obj.Ao(3) =  magpks;
            end
            Ao = obj.Ao;
            f = obj.f;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end

% Obtains the single-sided magnitude and phase spectra of the signal and
% the corresponding normalized frequencies
function [absX,angX,f] = dft_custom(x,N)
    dft = fft(x,N);
    dft = dft(1:N/2+1)/length(x);   % obtains only k between [0,N/2]
    dft(2:end-1) = 2*dft(2:end-1);  % do not double 0 or N/2
    f = 2*(0:length(dft)-1)/N;
    absX = abs(dft);
    angX = angle(dft);
end