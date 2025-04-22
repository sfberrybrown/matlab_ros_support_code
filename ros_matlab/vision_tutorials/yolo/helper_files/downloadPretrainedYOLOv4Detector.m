function detector = downloadPretrainedYOLOv4Detector()
% Download a pretrained yolov4 detector.
    if ~exist("yolov4TinyVehicleExample_24a.mat", "file")
        if ~exist("yolov4TinyVehicleExample_24a.zip", "file")
            disp("Downloading pretrained detector...");
            pretrainedURL = "https://ssd.mathworks.com/supportfiles/vision/data/yolov4TinyVehicleExample_24a.zip";
            websave("yolov4TinyVehicleExample_24a.zip", pretrainedURL);
        end
        unzip("yolov4TinyVehicleExample_24a.zip");
    end
    pretrained = load("yolov4TinyVehicleExample_24a.mat");
    detector = pretrained.detector;
end