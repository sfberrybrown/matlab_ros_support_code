function data = augmentDataKeepingAspectRatio(A,inputSize,doAugmentation)
% Data augmentation
% This function helps in increasing the training images by data augmentation
% Scaling and randomly altering color of the pixels are used in this script
targetSize = [inputSize(2),inputSize(1)];
data = cell(size(A));
for ii = 1:size(A,1)
    I = A{ii,1};
    bboxes = A{ii,2};
    labels = A{ii,3};
    sz = size(I);
    if numel(sz) < 3
        I = repmat(I,[1 1 3]);
    end

    if doAugmentation
        zoomRange = [0.8 1.5];
        rotateRange = deg2rad(0);
        translateRange = 0;
        flip = false;
    else
        zoomRange = [1 1];
        rotateRange = 0;
        translateRange = 0;
        flip = false;
    end

    centerXShift = sz(2)/2;
    centerYShift = sz(1)/2;
    [moveToOriginTransform,moveBackTransform] = deal(eye(3));
    moveToOriginTransform(3,1) = -centerXShift;
    moveToOriginTransform(3,2) = -centerYShift;
    moveBackTransform(3,1) = centerXShift;
    moveBackTransform(3,2) = centerYShift;

    zoomFactor = rand(1,1)*(zoomRange(2)-zoomRange(1))+zoomRange(1);
    scaleTransform = [zoomFactor 0 0;
        0 zoomFactor 0;
        0 0 1];

    rotationFactor = randn(1,1)*rotateRange;
    rotationFactor = min(max(rotationFactor,-rotateRange*2),rotateRange*2);
    rotationTransform = [cos(rotationFactor) sin(rotationFactor) 0;
        -sin(rotationFactor) cos(rotationFactor) 0;
        0 0 1];
    if flip && (rand > 0.5)
        x_scale = -1;
    else
        x_scale = 1;
    end
    reflectionTransform = [x_scale 0 0; 0 1 0; 0 0 1];

    translationTransform = eye(3);
    translationTransform(3,1) = randn*translateRange;
    translationTransform(3,2) = randn*translateRange;

    [maxEdge,ind] = max([sz(1),sz(2)]);
    scale = targetSize(ind)/maxEdge;
    scaledTransform = eye(3);
    scaledTransform(1,1) = scale;
    scaledTransform(2,2) = scale;

    moveToTopLeftTransform = eye(3);
    moveToTopLeftTransform(3,1) = (targetSize(2)/scale-sz(2))/2;
    moveToTopLeftTransform(3,2) = (targetSize(1)/scale-sz(1))/2;

    letterboxCropTransform = moveToTopLeftTransform*scaledTransform;

    centeredRotation = moveToOriginTransform * rotationTransform * moveBackTransform;
    centeredScale = moveToOriginTransform * scaleTransform * moveBackTransform;
    centeredReflection = moveToOriginTransform * reflectionTransform * moveBackTransform;

    Tout = centeredReflection * centeredScale * centeredRotation * translationTransform *  letterboxCropTransform;
    tform = affine2d(Tout);

    [Iout,rout] = imwarp(I,tform,...
        'OutputView',imref2d([targetSize(2) targetSize(1)]),...
        'SmoothEdges',true,...
        'FillValues',127);

    areaOrig = prod(bboxes(:,3:4),2);
    bboxesPoints = bbox2points(bboxes);
    bboxesPointsTrans = tform.transformPointsForward(reshape(permute(bboxesPoints,[1 3 2]),[],2));
    bboxesPointsTrans = permute(reshape(bboxesPointsTrans,4,[],2),[1 3 2]);
    bboxesPointsTrans = max(bboxesPointsTrans,1);
    bboxesPointsTrans = cat(2,min(bboxesPointsTrans(:,1,:),rout.ImageSize(1)),min(bboxesPointsTrans(:,2,:),rout.ImageSize(2)));
    bboxesTopLeftBotRight = cat(1,min(bboxesPointsTrans,[],1),max(bboxesPointsTrans,[],1));
    bboxesTrans = cat(2,bboxesTopLeftBotRight(1,:,:),bboxesTopLeftBotRight(2,:,:)-bboxesTopLeftBotRight(1,:,:)+1);
    bboxesOut = permute(bboxesTrans,[3 2 1]);
    areaOut = prod(bboxesOut(:,3:4),2);

    indices = find(areaOut > areaOrig * abs(det(tform.T)) * 0.75);
    bboxesOut = bboxesOut(indices,:);
    labelsOut = labels(indices);

    if isempty(indices)
        tform = affine2d(letterboxCropTransform);
        [Iout,rout] = imwarp(I,tform,...
            'OutputView',imref2d([targetSize(2) targetSize(1)]),...
            'SmoothEdges',true,...
            'FillValues',127);
        [bboxesOut,indices] = bboxwarp(round(bboxes),tform,rout,'OverlapThreshold',0.25);
        labelsOut = labels(indices);
    end

    if doAugmentation
        if numel(sz) == 3 && sz(3) == 3
            Iout = jitterColorHSV(Iout,...
                'Contrast',0.2,...
                'Hue',0.25,...
                'Saturation',0.2,...
                'Brightness',0.2);
        end
    end

    Iout = im2single(Iout);
    data(ii,:) = {Iout, bboxesOut, labelsOut};
end
end