import tkinter as tk
import params
import transformations

# Creates the GUI
def create_slider():
    # Nested function to write slider values to params file
    def show_overlay():
        params.usr_translation = (w1.get(), w2.get(), w3.get())
        params.usr_rotation = (w4.get(), w5.get(), w6.get())
        
    # Nested function to print out the slider values and total rot / trans values  as well as the calibation matrix to the terminal
    # Saves all the calibration parameters to a .txt file called calibrations.txt
    def print_calibration_parameters():
        translation_tot = ((params.translation[0]+w1.get()), (params.translation[1]+w2.get()),(params.translation[2]+w3.get()))
        rotation_tot = ((params.rotation[0]+w4.get()), (params.rotation[1]+w5.get()), (params.rotation[2]+w6.get()))
        matrix = transformations.matrix(rotation_tot, translation_tot)
        translation_gui = (w1.get(), w2.get(), w3.get()) 
        rotation_gui = (w4.get(), w5.get(), w6.get()) 
        print ("The translation you entered is:", translation_gui)
        print ("The rotation you entered is:", rotation_gui)
        print("The total calibration matrix is:")
        print (matrix[0])
        print (matrix[1])
        print (matrix[2])
        
        # Writing inputs to .txt file
        txt = open ("calibrations.txt", "a")
        txt.write("########################### \n")
        txt.write("\n")
        txt.write("The calibration matrix is: \n")
        txt.write(str(matrix[0]))
        txt.write("\n")
        txt.write(str(matrix[1]))
        txt.write("\n")
        txt.write(str(matrix[2]))
        txt.write("\n")
        txt.write("\n")
        txt.write("The initial translation is: \n")
        txt.write(str(params.translation))
        txt.write("\n")
        txt.write("The translation you entered is: \n")
        txt.write(str(translation_gui))
        txt.write("\n")
        txt.write("\n")
        txt.write("The initial rotation is: \n")
        txt.write(str(params.rotation))
        txt.write("\n")
        txt.write("The rotation you entered is: \n")
        txt.write(str(rotation_gui))
        txt.write("\n")


    # Definition of sliders
    master = tk.Tk()
    master.title("Calibration Tool")

    w1 = tk.Scale(master, label = "Translation along x-axis (meters):", from_ = params.min_trans, to = params.max_trans, \
        length = params.width_slider, orient = tk.HORIZONTAL, resolution = params.resolution_trans)
    w1.set(params.usr_translation[0])
    w1.pack()
    w2 = tk.Scale(master, label = "Translation along y-axis (meters):", from_ = params.min_trans, to = params.max_trans, \
        length = params.width_slider, orient = tk.HORIZONTAL, resolution = params.resolution_trans)
    w2.set(params.usr_translation[1])
    w2.pack()
    w3 = tk.Scale(master, label = "Translation along z-axis (meters):", from_ = params.min_trans, to = params.max_trans, \
        length = params.width_slider, orient = tk.HORIZONTAL, resolution = params.resolution_trans )
    w3.set(params.usr_translation[2])
    w3.pack()
    w4 = tk.Scale(master, label = "Rotation around x-axis (degrees) (Rotation order: z, y', x''):", \
        from_ = params.min_deg, to = params.max_deg, length = params.width_slider, orient = tk.HORIZONTAL, resolution = params.resolution_rot)
    w4.set(params.usr_rotation[0])
    w4.pack()
    w5 = tk.Scale(master, label = "Rotation around y-axis (degrees):", from_ = params.min_deg, to = params.max_deg, \
        length = params.width_slider, orient = tk.HORIZONTAL, resolution = params.resolution_rot)
    w5.set(params.usr_rotation[1])
    w5.pack()
    w6 = tk.Scale(master, label = "Rotation around z-axis (degrees):", from_ = params.min_deg, to = params.max_deg, \
        length = params.width_slider, orient = tk.HORIZONTAL, resolution = params.resolution_rot)
    w6.set(params.usr_rotation[2])
    w6.pack()

    # Definition of buttons
    tk.Button(master, text = '          Show Overlay          ', command = show_overlay).pack()
    tk.Button(master, text = 'Print Calibration Parameters', command = print_calibration_parameters).pack()
    tk.mainloop()
    

def main(args=None):
    create_slider()

if __name__ == '__main__':
    main()
