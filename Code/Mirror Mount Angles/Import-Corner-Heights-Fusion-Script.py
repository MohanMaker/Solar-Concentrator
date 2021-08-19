import adsk.core, adsk.fusion, adsk.cam, traceback
import csv

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        rootComp = design.rootComponent # root component of the active design
        
        ui.messageBox('Creating Designs from CSV')

        # folder to write out the results.
        folder = '/Users/mohanhathi/Desktop/Projects/Solar-Concentrator/angle-calc-stl-export/'

        # get the parameters names to change.
        corner1 = design.userParameters.itemByName('Corner1')
        corner2 = design.userParameters.itemByName('Corner2')
        corner3 = design.userParameters.itemByName('Corner3')
        corner4 = design.userParameters.itemByName('Corner4')

        with open('/Users/mohanhathi/Desktop/Projects/Solar-Concentrator/heliostat-angle-and-corner-height-calculation.csv', 'r', newline='') as csvfile:
                csvreader = csv.reader(csvfile)
                # iterate over the header row
                header = next(csvreader)
                # extracting the data from the other rows data row one by one
                for row in csvreader:
                    # change parameters
                    corner1.expression = row[1]
                    corner2.expression = row[2]
                    corner3.expression = row[3]
                    corner4.expression = row[4]

                    # change text
                    # get the sketch named "Identification Text"
                    sk = rootComp.sketches.itemByName('Row-Column Text')
                    # get the first sketch text.
                    skText = sk.sketchTexts.item(0)
                    # Change the text.
                    skText.text = row[0]

                    # allow view to have a chance to paint to watch progress.
                    adsk.doEvents()
                    design.computeAll()

                    # construct the output filename.
                    filename = folder + row[0] + '.stl'
                    # save the file as STL.
                    exportMgr = adsk.fusion.ExportManager.cast(design.exportManager)
                    stlOptions = exportMgr.createSTLExportOptions(rootComp)
                    stlOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
                    stlOptions.filename = filename
                    exportMgr.execute(stlOptions)
            
        ui.messageBox('Finished')

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))