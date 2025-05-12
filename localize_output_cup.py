#Import libraries
import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import os.path
import math
import statistics

def get_output_cup_location(bag_path=None, FileIO=False):
    cup_centroids = []
    cup_height = []
    cup_radius = []
    
    try:
        # Create pipeline
        pipeline = rs.pipeline()

        # Create a config object
        config = rs.config()
        
        if FileIO==True:
            # Tell config that we will use a recorded device from file to be used by the pipeline through playback.
            rs.config.enable_device_from_file(config, str(bag_path))
        else:
            config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        # Start streaming from file
        pipeline.start(config)
        align_to = rs.stream.color
        align = rs.align(align_to)
        
        cv2.namedWindow("Output Stream", cv2.WINDOW_AUTOSIZE)
        
        # Create colorizer object
        colorizer = rs.colorizer(2)

        fourcc = cv2.VideoWriter_fourcc(*'MP4V')
        out_vid = cv2.VideoWriter('Localization_output.mp4',fourcc, 30.0, (1280,720))

        # Streaming loop
        for i in range(20):
            # Get frameset of depth
            unalligned_frames = pipeline.wait_for_frames()
            frames = align.process(unalligned_frames)

            # Get depth frame
            rgb_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            intrinsics = depth_frame.get_profile().as_video_stream_profile().get_intrinsics()

            if depth_frame.get_data() == None:
                continue

            depth_color_frame = colorizer.colorize(depth_frame)
            base_depth_frame = np.asanyarray(depth_frame.get_data())
            depth_color_image = np.asanyarray(depth_color_frame.get_data())

            rgb_color_image = np.asanyarray(rgb_frame.get_data())
            rgb_color_image = cv2.cvtColor(rgb_color_image, cv2.COLOR_RGB2BGR)

            output_display = np.zeros((rgb_color_image.shape[0],256,3))
            overlay_img = cv2.imread("coffee.jpg")
            overlay_img = cv2.resize(overlay_img, (output_display.shape[1],output_display.shape[0]-200), interpolation=cv2.INTER_AREA)

            roi_tl =[455,97]
            roi_br = [755,400]
            roi = rgb_color_image[roi_tl[1]:roi_tl[0], roi_br[1]:roi_br[0],:]

            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) 
            # Blur using 3 * 3 kernel. 
            gray_blurred = cv2.blur(gray, (3, 3)) 
      
            # Apply Hough transform on the blurred image. 
            detected_circles = cv2.HoughCircles(gray_blurred,  
                            cv2.HOUGH_GRADIENT, 1, 100, param1 = 50, 
                        param2 = 30, minRadius = 10, maxRadius = 100) 
                        
            
            # Draw biggest detected circle within allowed range
            if detected_circles is not None: 
            
                detected_circles = np.uint16(np.around(detected_circles)) 
            
                for pt in detected_circles[0, :]: 
                    a, b, r = pt[0], pt[1], pt[2] 
            
                    # Draw the circumference of the circle. 
                    cv2.circle(rgb_color_image, (a+roi_br[1], b+roi_tl[1]), r, (0, 255, 0), 2)
                    overlay = depth_color_image.copy()
                    cv2.circle(overlay, (a+roi_br[1], b+roi_tl[1]), r, (255, 255, 50), -1)  # Draw filled circle on the overlay
                    center = base_depth_frame[b+roi_tl[1],a+roi_br[1]]
                    cup_patch = base_depth_frame[b+roi_tl[1]-r:b+roi_tl[1]+r,a+roi_br[1]-r:a+roi_br[1]+r]
                    values_in_range = cup_patch[(cup_patch >= 50) & (cup_patch <= 1000)]
                    max_height_value = np.max(values_in_range)
                    min_height_value = np.min(values_in_range)
                    height = (max_height_value - min_height_value)/1000 #height is in mm
                    cv2.addWeighted(overlay, 0.6, depth_color_image, 1 - 0.6, 0, depth_color_image) 
            
                    # Draw a small circle (of radius 1) to show the center. 
                    cv2.circle(rgb_color_image, (a+roi_br[1], b+roi_tl[1]), 1, (0, 0, 255), 3)
                    depth_value = depth_frame.get_distance(a+roi_br[1], b+roi_tl[1])
                    point = rs.rs2_deproject_pixel_to_point(intrinsics, [a+roi_br[1], b+roi_tl[1]], min_height_value/1000)
                    rim_point1 = rs.rs2_deproject_pixel_to_point(intrinsics, [a+roi_br[1]-r, b+roi_tl[1]], min_height_value/1000)
                    rim_point2 = rs.rs2_deproject_pixel_to_point(intrinsics, [a+roi_br[1]+r, b+roi_tl[1]], min_height_value/1000)
                    x1, y1, z1 = rim_point1
                    x2, y2, z2 = rim_point2
                    diameter = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

                    #add details
                    output_display = cv2.putText(output_display, "Position = ("+str(int(point[0]*1000))+" , "+str(int(point[1]*1000))+" , "+str(int(point[2]*1000))+")", (int(output_display.shape[1]/2)-125,60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50,255,255), 2, cv2.LINE_AA)
                    output_display = cv2.putText(output_display, "cup height = "+str(int(height*1000))+" mm", (int(output_display.shape[1]/2)-100,120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50,255,255), 2, cv2.LINE_AA)
                    output_display = cv2.putText(output_display, "lid radius = "+str(int((diameter/2)*1000))+" mm", (int(output_display.shape[1]/2)-100,180), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50,255,255), 2, cv2.LINE_AA)
                    output_display[200:,:,:] = overlay_img
                    break  
            else:
                output_display = cv2.putText(output_display, 'No', (int(output_display.shape[1]/2)-20,int(output_display.shape[0]/2)-60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
                output_display = cv2.putText(output_display, 'Coffee Mug', (int(output_display.shape[1]/2)-90,int(output_display.shape[0]/2)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
                output_display = cv2.putText(output_display, 'Placed!!', (int(output_display.shape[1]/2)-60,int(output_display.shape[0]/2)+60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)


            # Render image in opencv window
            depth_color_image_crped = depth_color_image[:,400:758,:]
            rgb_color_image[:output_display.shape[0],:output_display.shape[1],:] = output_display
            rgb_color_image[:depth_color_image_crped.shape[0],rgb_color_image.shape[1]-depth_color_image_crped.shape[1]:,:] = depth_color_image_crped
            out_vid.write(rgb_color_image)
            cv2.imshow("Output Stream", rgb_color_image)
            key = cv2.waitKey(1)
            # if pressed escape exit program
            if key == 27:
                cv2.destroyAllWindows()
                out_vid.release()
                break
            cup_centroids.append(point)
            cup_height.append(height)
            cup_radius.append(diameter/2)
    finally:
        pass
    if len(cup_centroids)==0:
        return None, None, None
    coordinates_array = np.array(cup_centroids)
    #accumulating mean
    mean_coordinates = np.mean(coordinates_array, axis=0)
    mean_height = statistics.mean(cup_height)
    mean_radius = statistics.mean(cup_radius)
    return mean_coordinates,mean_height, mean_radius

def main():
    parser = argparse.ArgumentParser(description="Read recorded bag file.")
    parser.add_argument("-i", "--input", type=str, help="Path to the bag file")
    args = parser.parse_args()
    # Safety if no parameter have been given
    if not args.input:
        print("No input paramater have been given.")
        print("For help type --help")
        exit()
    # Check if the given file have bag extension
    if os.path.splitext(args.input)[1] != ".bag":
        print("The given file is not of correct file format.")
        print("Only .bag files are accepted")
        exit()
    point,height,radius=get_output_cup_location(bag_path=args.input, FileIO=True)
    print(f"Output : Height = {height}, radius = {radius}, lid_centroid={point}")
    

if __name__ == "__main__":
    main()