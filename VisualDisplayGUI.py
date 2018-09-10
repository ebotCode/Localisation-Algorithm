"""
Author : Tobe 

VisualDisplayGUI.py 

"""

from tkinter import *
import time 

import os

from VisualDisplay import VisualObject, ParticleObject

class DisplayScreen(Frame):
    """
    a canvas based gui that allows for the sampling of points from a picture
    and generates the coordinates in a file.
    * The Screen is to be caliberated to the image dimensions that you want
      to sample. This is done with the first two points, after which 'caliberate'
      is to be clicked, then the screen is caliberated.
    * to sample points, just click on it. 



    """
    def __init__(self,bbox = None, object_dict = None, simulation_dict = None ):
        """ args:
    bbox: is list of the bounding box rectangle. 
            bbox = [(x_left_top_corner,y_left_top_corner),
                    (x_right_bottom_corner,y_right_bottom_corner)]
    object_list: list of visualObjects 
    """
        # initialise frame 
        Frame.__init__(self)
        # configure row and colums with title 
        self.master.rowconfigure(0,weight = 1)
        self.master.columnconfigure(0,weight = 1)
        self.master.title('Robot DisplayScreen')
        self.configure(bg = 'black')
        # grid the frame to display 
        self.grid(row = 0,column = 0,sticky = W+E+N+S)
        # set the cell (0,0) to expand faster. 
        self.rowconfigure(0,weight = 1)
        self.columnconfigure(0,weight = 1)
        # set the geometry of the window 
        self.master.geometry('%dx%d'%(800,600))
        # initialise caliberation parameters
        self.bbox = bbox
        self.xmin_actual = 0 
        self.xmax_actual = 0
        self.ymin_actual = 0 
        self.ymax_actual = 0 

        self.xmin_pixel = 0
        self.xmax_pixel = 0
        self.ymin_pixel = 0
        self.ymax_pixel = 0

        self.object_dict = None 
        self.simulation_dict = simulation_dict 
        self.timer = None 
        #  initialise canvas 
        self.mycanvas = None 
        self.mycanvas_items_idtag = []
        self.createCanvas()     
        self.setObjects(object_dict)

        # :: saved simulation results 
        self.saved_result_index = 0 
        self.saved_simulation_result = []         
        self.number_of_saved_results = 0  
        
     
        
    def createCanvas(self):
        # create canvas
        self.mycanvas = Canvas(self,bg = 'white')
        self.mycanvas.grid(row = 0,column = 0, sticky = W+E+N+S,pady = 0)
        # self.mycanvas.create_image(0,0,image = self.image,anchor = 'nw')
        self.mycanvas_items = [];
        # boind mouse event to canvas
        self.mycanvas.bind('<Motion>',self.showCoordinate)
        self.mycanvas.bind('<Button-1>',self.markCoordinate)

        # create status text
        self.status_text = 'ready: ==> pixel = (%.4f,%.4f) , coordinate = (%.4f %.4f) '

        # status frame
        self.statusframe = Frame(self,bg = 'white')
        self.statusframe.rowconfigure(0,weight = 1)
        self.statusframe.columnconfigure(1,weight = 1)
        self.statusframe.grid(row= 1, column = 0, sticky = W+E+N+S)
        
        # status bar
        self.mystatus = Label(self.statusframe,bg = 'white')
        self.mystatus.config(text = self.status_text)
        self.mystatus.grid(row = 0, column = 1,sticky = W+E+N+S)


        # save points
        self.saveButton = Button(self.statusframe,text = 'Caliberate',command = self.saveCoordinates)
        self.saveButton.grid(row = 0,column = 0, sticky = W+E+N+S)

        # caliberation pane
        self.calibpane = Frame(self,bg = "green")
        self.calibpane.columnconfigure(0,weight = 1)

        self.caliberate() 



    def caliberate(self):
        """ sets the caliberation parameters for conversion from pixel to acutal coordinate and 
            vice versa. 
            Note:   
                xmin_pixel corresponds to the pixel value where x_min is located (left)
                xmax_pixel corresponds to the pixel value where x_max is located (right)
                ymin_pixel corresponds to the pixel value where y_min is located (bottom) 
                ymax_pixel corresponds to the pixel value where y_max is located (top)
        """
        bbox_coordinate = self.bbox 
        bounding_margin = 5

        top_left_corner = bbox_coordinate[0]
        bottom_right_corner = bbox_coordinate[1]

        canvas_width,canvas_height = self.getCanvasSize()

        self.xmin_actual = top_left_corner[0]
        self.xmin_pixel  = bounding_margin 

        self.xmax_actual = bottom_right_corner[0]
        self.xmax_pixel  = 800 #canvas_width 

        self.ymin_actual = bottom_right_corner[1]
        self.ymin_pixel  = 570# canvas_height 

        self.ymax_actual = top_left_corner[1]
        self.ymax_pixel  = bounding_margin  

        print('xmin - %f, %f'%(self.xmin_actual,self.xmin_pixel))
        print('xmax - %f, %f'%(self.xmax_actual,self.xmax_pixel))
        print('ymin - %f, %f'%(self.ymin_actual,self.ymin_pixel))
        print('ymax - %f, %f'%(self.ymax_actual,self.ymax_pixel))
        print('canvas width ',canvas_width)
        print('canvas height ', canvas_height) 
                   

    def convertToActual(self,point_pixel):
        """ returns the conversion from pixel to actual coordinate. if the
        pixel coordinates bounding rectangle have not been set, it returns the pixel coordiates
        as the actual coordinates"""
        xp,yp= point_pixel 
        xactual = self.interpolate(xp,[self.xmin_pixel,self.xmax_pixel],[self.xmin_actual,self.xmax_actual])
            
        yactual = self.interpolate(yp,[self.ymin_pixel,self.ymax_pixel],[self.ymin_actual,self.ymax_actual])
            
        return (xactual,yactual)

    def convertToPixel(self,point_actual):
        """ returns the a tuple the conversion from actual point to pixel coordinate as tuple. """ 
        xactual,yactual = point_actual 
        xpixel = self.interpolate(xactual,[self.xmin_actual,self.xmax_actual],[self.xmin_pixel,self.xmax_pixel])
        ypixel = self.interpolate(yactual,[self.ymin_actual,self.ymax_actual],[self.ymin_pixel,self.ymax_pixel])

        return(xpixel,ypixel)

    def drawObjects(self):
        if self.object_dict: 
            # first clear the canvas screen 
            self.mycanvas.delete('all')
            # redraw all objects 
            for object_name in self.object_dict:
                if object_name == 'particles':
                    for particle in self.object_dict[object_name]:
                        particle.drawObject(self)
                else:
                    self.object_dict[object_name].drawObject(self)

                # line_segments = self.object_dict[object_name].getPoints()
                # object_color  = self.object_dict[object_name].getColor()
                # for line in line_segments:
                #     p1 = self.convertToPixel(line[0])
                #     p2 = self.convertToPixel(line[1])
                #     self.mycanvas.create_line(p1[0],p1[1],p2[0],p2[1],fill = object_color)

    def displayRectangle(self,top_left_corner,bottom_right_corner,fill = 'black'):
        """ displays a rectangle. all coordinates are supplied in mm """
        p1 = self.convertToPixel(top_left_corner)
        p2 = self.convertToPixel(bottom_right_corner)
        self.mycanvas.create_rectangle(p1[0],p1[1],p2[0],p2[1],fill = fill)  

    def displayLine(self,p1,p2,fill = 'black'):  
        p1 = self.convertToPixel(p1)
        p2 = self.convertToPixel(p2)   
        self.mycanvas.create_line(p1[0],p1[1],p2[0],p2[1],fill = fill)  

    def displayOval(self,p1,p2,fill = 'black'):
        p1 = self.convertToPixel(p1)
        p2 = self.convertToPixel(p2)   
        self.mycanvas.create_oval(p1[0],p1[1],p2[0],p2[1],fill = fill)  

        
    def getCanvasSize(self):
        """ returns the size of the canvas in pixel"""
        return (self.mycanvas.winfo_width(),self.mycanvas.winfo_height())

    def interpolate(self,x,x_lim,y_lim):
        """ returns the result of a liner interpolation for the unknown value.
            x_lim[0]  ------ y_lim[0]
                |               |
                x               y
                |               |
            x_lim[1]  ------ y_lim[1]
            """
        numer = (x - x_lim[1])
        denom = (x_lim[0] - x_lim[1])
        dy    = (y_lim[0] - y_lim[1])

        if abs(denom) > 0:
            y = y_lim[1] + (float(numer)/denom)*dy 
        else: # if denominator is 0 return x as the interpolation of y 
            y = x 

        return y 


    def markCoordinate(self,event = None):
        """ marks the coordinate at the current mouse click position"""
        x1,y1 = event.x,event.y
        idtag = self.mycanvas.create_oval((x1 -2),(y1-2),(x1 +2),(y1 + 2),fill = 'black')
        # self.mycanvas_items_tags.append(idtag)
          

    def saveCoordinates(self,event = None):
        """ prints actual coordinates present in actual_coord"""
        pass
   
    def showCoordinate(self,event):
        """ show the coordinates of the current position of the mouse"""
        p1 = event.x,event.y
        p2 = self.convertToActual(p1)
        self.setStatusText(p1,p2)

        # set status text

    def setStatusText(self,p1,p2):
        self.mystatus.config(text = self.status_text%(p1[0],p1[1],p2[0],p2[1]))


    def setObjects(self,object_dict):
        """ sets the dict of objects that would be drawn """
        self.object_dict = object_dict 

    def runSavedSimulation2(self):
        if self.saved_result_index < self.number_of_saved_results:
            [error,sensor_ray_segs,particles_position] = self.saved_simulation_result[self.saved_result_index]

            particles = [ParticleObject(position) for position in particles_position]
            self.object_dict['particles'] = particles
            self.object_dict['sensor'].setSequence(sensor_ray_segs) # set the particle sequence 
            self.drawObjects()
            print("Error = %.4f"%error)
            self.saved_result_index += 1 
            self.mycanvas.after(100,self.runSavedSimulation2)

    def runSavedSimulation(self,simulation_result):
        """ displays the list of simulation_results 
            each item in simulation_result contains 
            [error,sensor_ray_segs,particles_position] 
        """
        self.saved_result_index = 0 
        self.saved_simulation_result = simulation_result 
        self.number_of_saved_results = len(simulation_result)

        self.runSavedSimulation2()




    def runSimulation(self,interval_display = 20,start = 0):
        # :: retrieve the filter_algorithm object
        filter_algorithm = self.simulation_dict["filter_algorithm"]
        # :: localise 
        sensor_ray_segs,error = filter_algorithm.localise(1) 
        # :: retrieve all particles anc create particles display object
        particles = [ParticleObject(particle.getPosition()) for particle in filter_algorithm.getParticles()]
        self.object_dict['particles'] = particles
        self.object_dict['sensor'].setSequence(sensor_ray_segs) # set the particle sequence 
        self.drawObjects()
        self.mycanvas.after(50,self.runSimulation)
        if error < 1e-2:
            # self.mycanvas.after_cancel
            print('CONVERGENCE REACHED')
            input('<enter to continue>')


        # self.after('cancel',self.timer)
        
        # _,sensor_ray_segs  = filter_algorithm.robot.measure()
        # for i in range(1,500):

        #     if i % interval_display == 0 : 
        #         #:: display for visual inspection 
        #             # retrieve the list of particles 
        #         particles = [ParticleObject(particle.getPosition()) for particle in filter_algorithm.getParticles()]
        #         self.object_dict['particles'] = particles
        #         self.object_dict['sensor'].setSequence(sensor_ray_segs)
        #         self.drawObjects()
        #         if i > 0 :
        #             input('<enter to continue>')
        #    # first call the localise function for the filter algorithm 
        #     sensor_ray_segs,error = filter_algorithm.localise(i) 
        #     if error < 1e-2:
        #         print("CONVERGENCE REACHED")
        #         break 
        # print (" CONVERGENCE "*20)    


    def runSimulation2(self,myrobot,p,computeWeights,resample,evalError):
        N = len(p) 
        for t in range(1000):

            ## :: make display here 
            if t % 20 == 0:
                rbt = self.simulation_dict['robot']
                rbt.setPosition(myrobot.getPosition())
                rbt.setOrientation(myrobot.getOrientation())
                _,sensor_ray_segs = rbt.measure()  
                          
                particles = [ParticleObject(particle.getPosition()) for particle in p]
                self.object_dict['particles'] = particles
                self.object_dict['sensor'].setSequence(sensor_ray_segs)
                self.drawObjects()
                if t > 0 :
                    print ("t = ",t)
                    input('<enter to continue>')

            myrobot = myrobot.move(0.1, 5.0)
            Z = myrobot.sense()
            # :: Make particles move withsame motion 
            p2 = []
            for i in range(N):
                p2.append(p[i].move(0.1, 5.0))
            p = p2
            # :: Compute weights for particles 
            w = computeWeights(Z,p)
            # :: Resample 
            p = resample(w,p,N)

            error_value = evalError(myrobot,p)
            print("*"*50)
            print("Error = ",error_value)
            print("*"*50)            

        # print (w)            
 

      
        
        
        
        

class SamplePoints:
    """
    loads a picture of a graph, and digitizes it, based on the
    caliberation given by the user
    """
    def __init__(self):
        pass

def main(bbox):
    """ runs the gui.
    Args: directory: common directory that contians the image to load
          dd_name: the name of the image file as seen in the directory
"""
    pass 

if __name__ == '__main__':
    main([(0,10),(10,0)])