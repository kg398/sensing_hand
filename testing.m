%net = resetState(net);

t = tcpclient('127.0.0.1',10000);
%x = linspace(1,200,200);
%plt=figure(1);
while 1
    %hold off
    %clf(plt)
    %hold on
    %line([50 150],[1 1],'Color','red','LineWidth',5)
    %line([50 150],[0 0],'Color','green','LineWidth',5)
    %ylim([-0.2 1.2]);
    while t.NumBytesAvailable == 0
    end
    ipt = readline(t)
    if ipt == "start"
%         for i=1:200
%             tic
%             X = char(readline(t));
%             if X == "stop"
%                 break
%             end
%             X = sscanf(X,'[%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f]');
%             X=X - muX ./ sigmaX;
%             [net,Z(i)] = predictAndUpdateState(net,X(:,:));
%             write(t,num2str(Z(i)));
%             %write(t,num2str(i/100.0))
%             toc
%         end

        net = resetState(net);
        %muX=mean(reshape(pose,[2000,32]))';
        %sigmaX=squeeze(std(std(pose(:,:,:))));
        clear X Z
        write(t,'ready')
        
        for i=1:50
            sdata=sscanf(char(readline(t)),'[%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f]');
            X(:,i)=sdata(:);
            if i==20
                livemuX = mean(X(:,1:20),2);  
            end
        end
        X=(X - livemuX) ./ sigmaX;
        [net,Z(:)] = predictAndUpdateState(net,X(:,:));
        for i=51:150
            sdata=sscanf(char(readline(t)),'[%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f]');
            X(:,i)=sdata(:);
            X(:,i)=(X(:,i) - livemuX) ./ sigmaX;
            [net,Z(i)] = predictAndUpdateState(net,X(:,i));
            write(t,num2str(Z(i)));

            %if sdata(33)==-1
            %plot(x(50:i),Z(50:i) ,'-b','LineWidth',8)
            %elseif sdata(33)==0
            %    plot(x(50:i),Z(50:i) ,'-g','LineWidth',8)
            %elseif sdata(33)==1
            %    plot(x(50:i),Z(50:i) ,'-r','LineWidth',8)
            %end

        end
    end
end



%for i=1:200
%    X=readline()
%    X=X - muX ./ sigmaX;
%    [net,Z(i)] = predictAndUpdateState(net,X(:,:));
%end