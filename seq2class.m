numChannels=32;
len=100;
%state(state>1)=1;
%asd=XTrain{1,1};
len_seq=190;


for j=1:100
    for i=1:len_seq
        data{i+((j-1)*len_seq)}=sens(1:len+randi([-49,49]),:,i);
        %data{i}=sens(1:len,((max(asd')-min(asd'))>0.4),i);
        TTrain(i+((j-1)*len_seq))=state(i);
    end

end



for n = 1:numel(data)
    X = data{n}';
    XTrain{n} = X(:,:);
    
end

muX = mean(cat(2,XTrain{:}),2);
sigmaX = std(cat(2,XTrain{:}),0,2);



for n = 1:numel(XTrain)
    XTrain{n} = (XTrain{n} - muX) ./ sigmaX;
 
end

sequenceLengths = cellfun(@(X) size(X,2), XTrain);
[sequenceLengthsSorted,idx] = sort(sequenceLengths);
XTrain = XTrain(idx);

TTrain = TTrain(idx);



layers = [ ...
    sequenceInputLayer(numChannels, Normalization="zscore")
    lstmLayer(40,'OutputMode','sequence')
    dropoutLayer(0.5)
    lstmLayer(30, OutputMode="last")
    fullyConnectedLayer(1)
    regressionLayer];


options = trainingOptions("adam", ...
    MaxEpochs=2000, ...
    InitialLearnRate=0.02*1e-2, ...
    SequencePaddingDirection="left", ...
    LearnRateDropPeriod=80,...
    LearnRateDropFactor=0.5,...
    LearnRateSchedule="piecewise", ...
    Shuffle="every-epoch", ...
    Plots="training-progress", ...
    Verbose=0);




net = trainNetwork(XTrain(:),TTrain(:),layers,options);


%
% for i=1:99
%
%     idx = i;
%     X = XTrain{idx};
%     T = TTrain{idx};
%
%
%     net = resetState(net);
%
%     offset=40;
%     [net,Z] = predictAndUpdateState(net,X(:,1:offset));
%
%
%     numPredictionTimeSteps = 80;
%
%
%     for t = 1:numPredictionTimeSteps
%         [net,Z(:,offset+t)] = predictAndUpdateState(net,Z(:,offset+t-1));
%     end
%
%     Zt=TTrain{idx};
%
%     err=Z-Zt(:,1:offset+numPredictionTimeSteps);
%     errav(:,i)=(rssq(err));
%
%     errall(i)=mean(rssq(err));
%
%     if state(i)==0
%         plot(errav(:,i),'b')
%        %plot(err(18,:),'b')
%         hold on
%     else
%        plot(errav(:,i),'r')
%       %plot(err(18,:),'r')
%         hold on
%     end
% end
%
%
%
% figure;
%
% for i=1:99
%
%     idx = i;
%     X = XTrain{idx};
%     T = TTrain{idx};
%
%
%     net = resetState(net);
%
%     offset=20;
%     [net,Z] = predictAndUpdateState(net,X(:,1:offset));
%
%
%     numPredictionTimeSteps = 100;
%
%
%     for t = 1:numPredictionTimeSteps
%         [net,Z(:,offset+t)] = predictAndUpdateState(net,X(:,offset+t-1));
%     end
%
%     Zt=TTrain{idx};
%
%     err=Z-Zt(:,1:offset+numPredictionTimeSteps);
%     errav(:,i)=(rssq(err));
%
%     errall(i)=mean(rssq(err));
%
%     if state(i)==0
%        plot(errav(:,i),'b')
%       %plot(err(6,:),'b')
%         hold on
%     else
%         plot(errav(:,i),'r')
%       %plot(err(6,:),'r')
%         hold on
%     end
% end


% for i=1:99
%
%     if state(i)==0
%
%          plot(XTrain{1, i}(18,:),'b')
%         hold on
%     else
%        plot(XTrain{1, i}(18,:),'r')
%         hold on
%     end
%
%
% end
%
%
%


% for i=1:99
%
%     if state(i)==0
%
%          plot(sens(:,18,i),'b')
%         hold on
%     else
%        plot(sens(:,18,i),'r')
%         hold on
%     end
%
%
% end


clear data
len=150;
for i=1:199
    data{i}=sens(1:len,:,i);
    %data{i}=sens(1:len,((max(asd')-min(asd'))>0.4),i);
end

for n = 1:numel(data)
    X = data{n}';
    XTrain{n} = X(:,1:len-1);
end




for n = 1:numel(XTrain)
    XTrain{n} = (XTrain{n} - muX) ./ sigmaX;
 
end


figure;

for i=190:199

    idx = i;
    X = XTrain{idx};



    net = resetState(net);

    offset=60;
    [net,Z] = predictAndUpdateState(net,X(:,1:offset));

    len=length(X);
    numPredictionTimeSteps = len-offset-1;


    for t = 1:numPredictionTimeSteps
        [net,Z(:,offset+t)] = predictAndUpdateState(net,X(:,offset+t-1));
    end



    if state(i)==0
        plot(Z,'b')
        %plot(err(6,:),'b')
        hold on
    else
        plot(Z,'r')
        %plot(err(6,:),'r')
        hold on
    end
end
