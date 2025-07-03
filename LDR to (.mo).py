ldraw_file = 'Gear_wheel1.ldr'  # LeoCAD file name (changeable)

model_title = ldraw_file[:-4]


def parse_ldraw_into_modelica(ldraw_file, model_title):


    part_construction = []
    connection_construction = []

    # Parsing process starts to extract and save position and rotation info
    with (open(ldraw_file, 'r') as file):
        modelica_code_part = f"model {model_title}\n"
        modelica_code_connection = "equation\n"

        gear_train = {}

        preX, preY, preZ, Rpre = 0, 0, 0, 0
        final_move_X, final_move_Y, final_move_Z = 0, 0, 0
        newx, newy, newz= 0, 0, 0


        gear_detector =0
        for line in file:

            line = line.strip().split()


            if line[0] == '1' and line[1] == '7':
                part_id = line[14]
                position = list(map(float, line[2:5]))
                rotation = list(map(float, line[5:14]))

                gear_detector += 1
                # append info to the list of all the gears
                gear_train[gear_detector]= {
                    'part_id': part_id,
                    'position': position,
                    'rotation': rotation,
                }



        gear_char = {  # this dictionary should include parts' specifications (adjustable)
            '6542a.dat': {"Diameter": 0.0175, "depth": 0.008, "ratio": "N/A"},
            '2740.dat': {"Diameter": 0.0259, "depth": 0.0077, "ratio": 1.48},
            '65413.dat': {"Diameter": 0.03, "depth": 0.008, "ratio": 1.1583011583011583011583011583012},
            '34432.dat': {"Diameter": 0.0482, "depth": 0.008, "ratio": 1.6066666666666666666666666666667},
            '60.dat': {"Diameter": 0.0254, "depth": 0.008, "ratio": 0.52697095435684647302904564315353}
        }

        prevDia = 1

        precedent_gears_info = {}
        # operation starts
        for idx, Rel_info in gear_train.items():     # So that we can access the value of the index itself and the value in that index /

            part_id = Rel_info['part_id']
            if part_id in gear_char:

                Rel_info['Diameter'] = gear_char[part_id]['Diameter']

                gear_ratio = Rel_info['Diameter'] / prevDia

                Rel_info['Depth'] = gear_char[part_id]['depth']


            currX = Rel_info['position'][0]
            # real_currX = x_position0.0004
            currZ = Rel_info['position'][1]*-1
            # real_currZ = z_position*0.0004
            currY = Rel_info['position'][2]
            # real_currY = y_position*0.0004


            first_rotation = Rel_info['rotation'][0]
            second_rotation = Rel_info['rotation'][1]
            third_rotation = Rel_info['rotation'][2]
            fourth_rotation = Rel_info['rotation'][3]
            fifth_rotation = Rel_info['rotation'][4]
            sixth_rotation = Rel_info['rotation'][5]
            seventh_rotation = Rel_info['rotation'][6]
            eighth_rotation = Rel_info['rotation'][7]
            ninth_rotation = Rel_info['rotation'][8]



            gear_dia = Rel_info["Diameter"] * (1000)
            Rcurr = gear_dia * (0.5)
            gear_thick = Rel_info["Depth"]
            gear_id = idx                                         # the id of the part "gear" in the pre-constructed dictionary based on part name in LeoCAD
            balance = gear_id - 1
            prevDia = Rel_info['Diameter']

            body_name = f"cyl{gear_id}"
            body_position_name = f"fixedTranslation{gear_id}"
            actuate_revolute_name = f"actuatedRevolute_{gear_id}"
            default_rotation_axis_y_tox = '1'

            last_idx = len(gear_train)



            if gear_id == 1:

                modelica_code_part += "  inner Modelica.Mechanics.MultiBody.World world(nominalLength = 100);\n"
                modelica_code_part += "  Modelica.Blocks.Sources.Constant const(k = 1);\n"
                modelica_code_part += "  Modelica.Mechanics.Rotational.Sources.Speed speed;\n"
                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {currX}}});\n"
                preX, preY, preZ, Rpre = currX, currY, currZ, Rcurr
                modelica_code_connection += f"  connect({body_position_name}.frame_a, world.frame_b);\n"
                modelica_code_connection += f"  connect({body_position_name}.frame_b, {actuate_revolute_name}.frame_a);\n"
                modelica_code_connection += f"  connect({actuate_revolute_name}.frame_b, {body_name}.frame_a);\n"
                modelica_code_connection += f"  connect(speed.flange, {actuate_revolute_name}.axis);\n"
                modelica_code_connection += f"  connect(speed.w_ref, const.y);\n"

                #Case#1 default_orientation (the gear rotates around Y-Axis in LeoCAD & X-Axis in OM)
                if ninth_rotation == 1 or ninth_rotation == -1:
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{{default_rotation_axis_y_tox}, 0, 0}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{{gear_thick}, 0, 0}});\n"
                    prev_rotation_axis = "Y"

                #Case#2 rotation around z(y in modelica)
                if eighth_rotation == 1 or eighth_rotation == -1:
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{0, {default_rotation_axis_y_tox}, 0}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{0, {gear_thick}, 0}});\n"
                    prev_rotation_axis = "Z"

                #Case#3 rotation around x(z in modelica)
                elif seventh_rotation == 1 or seventh_rotation == -1:
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{0, 0, {default_rotation_axis_y_tox}}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{0,0, {gear_thick}}});\n"
                    prev_rotation_axis = "X"



            if gear_id != last_idx and gear_id != 1:


                modelica_code_part += f"  Modelica.Mechanics.Rotational.Components.IdealGear idealGear{balance}(ratio = {gear_ratio});\n"
                modelica_code_connection += f"  connect({body_position_name}.frame_a, world.frame_b);\n"
                modelica_code_connection += f"  connect({body_position_name}.frame_b, {actuate_revolute_name}.frame_a);\n"
                modelica_code_connection += f"  connect({actuate_revolute_name}.frame_b, {body_name}.frame_a);\n"
                modelica_code_connection += f"  connect(idealGear{balance}.flange_a, actuatedRevolute_{balance}.axis);\n"
                modelica_code_connection += f"  connect(idealGear{balance}.flange_b, {actuate_revolute_name}.axis);\n"

                #Case#1 default_orientation (the gear rotates around Y-Axis in LeoCAD & X-Axis in OM)
                if ninth_rotation == 1 or ninth_rotation == -1:
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{{default_rotation_axis_y_tox}, 0, 0}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{{gear_thick}, 0, 0}});\n"

                    # Non-Mating case
                    if prev_rotation_axis == "Y" and preY != currY:
                        if preX == currX and preZ == currZ:
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, currZ, Rcurr
                            prev_rotation_axis = "Y"


                        elif preX != currX:

                            newx = currX + final_move_X

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"

                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                            oldX, oldY, oldZ = currX, currY, currZ

                            final_move_X = preX - oldX

                            final_move_Z = preZ - oldZ

                            prev_rotation_axis = "Y"



                        elif preZ != currZ:

                            newz = currZ + final_move_Z

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                            oldX, oldY, oldZ = currX, currY, currZ

                            final_move_X = preX - oldX

                            final_move_Z = preZ - oldZ

                            prev_rotation_axis = "Y"

                        # Mating Gear

                    elif preX != currX and preY == currY and preZ == currZ:

                        gear_values = [precedent_gears_info[k] for k in sorted(precedent_gears_info)]

                        skip_post_for = False

                        for gt_key, gp_info in enumerate(gear_values, start=1):

                            oldVY = gp_info['Y']

                            oldVX = gp_info['X']

                            oldVZ = gp_info['Z']

                            oldFMX = gp_info['FinalMX']

                            oldFMZ = gp_info['FinalMZ']

                            oldR = gp_info['R']

                            if gt_key <= balance - 1:

                                if currY == oldVY:

                                    if currZ != oldVZ and currX != oldVX:

                                        if abs(currX - oldVX) > abs(currZ - oldVZ):

                                            mating_orientation = "horizontally mated"

                                            if oldFMZ != 0 and oldFMX == 0:

                                                newz = currZ + oldFMZ

                                                if currX > oldVX:

                                                    newx = oldVX + oldR + Rcurr

                                                elif currX < oldVX:

                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMZ == 0 and oldFMX != 0:

                                                if currX > oldVX:

                                                    newx = oldVX + oldR + Rcurr + oldFMX

                                                elif currX < oldVX:

                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMZ != 0 and oldFMX != 0:

                                                newz = currZ + oldFMZ

                                                if currX > oldVX:

                                                    newx = oldVX + oldR + Rcurr + oldFMX

                                                elif currX < oldVX:

                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMZ == 0 and oldVX == 0:

                                                if currX > oldVX:

                                                    newx = oldVX + oldR + Rcurr

                                                elif currX < oldVX:

                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                        elif abs(currZ - oldVZ) > abs(currX - oldVX):

                                            mating_orientation = "vertically mated"

                                            if oldFMX != 0 and oldFMZ == 0:

                                                newx = currX + oldFMX

                                                if currZ > oldVZ:

                                                    newz = oldVZ + oldR + Rcurr

                                                elif currZ < oldVZ:

                                                    newz = oldVZ - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMX == 0 and oldFMZ != 0:

                                                if currZ > oldVZ:

                                                    newz = oldVZ + oldR + Rcurr + oldFMZ

                                                elif currZ < oldVZ:

                                                    newz = oldVZ - oldR - Rcurr + oldFMZ

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMZ != 0 and oldFMX != 0:

                                                newx = currX + oldFMX

                                                if currZ > oldVZ:

                                                    newz = oldVZ + oldR + Rcurr + oldFMZ

                                                elif currZ < oldVZ:

                                                    newz = oldVZ - oldR - Rcurr + oldFMZ

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMZ == 0 and oldVX == 0:

                                                if currZ > oldVZ:

                                                    newz = oldVZ + oldR + Rcurr

                                                elif currZ < oldVZ:

                                                    newz = oldVZ - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True

                                    if oldVX != currX and oldVZ == currZ:

                                        if oldFMZ != 0 and oldFMX == 0:

                                            newz = currZ + oldFMZ

                                            if currX > oldVX:

                                                newx = oldVX + oldR + Rcurr

                                            elif currX < oldVX:

                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMZ == 0 and oldFMX != 0:

                                            if currX > oldVX:

                                                newx = oldVX + oldR + Rcurr + oldFMX

                                            elif currX < oldVX:

                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMZ != 0 and oldFMX != 0:

                                            newz = currZ + oldFMZ

                                            if currX > oldVX:

                                                newx = oldVX + oldR + Rcurr + oldFMX

                                            elif currX < oldVX:

                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMZ == 0 and oldVX == 0:

                                            if currX > oldVX:

                                                newx = oldVX + oldR + Rcurr

                                            elif currX < oldVX:

                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True

                                    if oldVX == currX and oldVZ != currZ:

                                        if oldFMX != 0 and oldFMZ == 0:

                                            newx = currX + oldFMX

                                            if currZ > oldVZ:

                                                newz = oldVZ + oldR + Rcurr

                                            elif currZ < oldVZ:

                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMX == 0 and oldFMZ != 0:

                                            if currZ > oldVZ:

                                                newz = oldVZ + oldR + Rcurr + oldFMZ

                                            elif currZ < oldVZ:

                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMZ != 0 and oldFMX != 0:

                                            newx = currX + oldFMX

                                            if currZ > oldVZ:

                                                newz = oldVZ + oldR + Rcurr + oldFMZ

                                            elif currZ < oldVZ:

                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMZ == 0 and oldVX == 0:

                                            if currZ > oldVZ:

                                                newz = oldVZ + oldR + Rcurr

                                            elif currZ < oldVZ:

                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True

                        if not skip_post_for:

                            if final_move_Z != 0 and final_move_X == 0:

                                newz = currZ + final_move_Z

                                if currX > preX:

                                    newx = preX + Rpre + Rcurr

                                elif currX < preX:

                                    newx = preX - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                oldX, oldY, oldZ = currX, currY, currZ

                                final_move_X = preX - oldX

                                final_move_Z = preZ - oldZ

                                prev_rotation_axis = "Y"



                            elif final_move_Z == 0 and final_move_X != 0:

                                if currX > preX:

                                    newx = preX + Rpre + Rcurr + final_move_X

                                elif currX < preX:

                                    newx = preX - Rpre - Rcurr + final_move_X

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"

                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                                oldX, oldY, oldZ = currX, currY, currZ

                                final_move_X = preX - oldX

                                final_move_Z = preZ - oldZ

                                prev_rotation_axis = "Y"


                            elif final_move_Z != 0 and final_move_X != 0:

                                newz = currZ + final_move_Z

                                if currX > preX:

                                    newx = preX + Rpre + Rcurr + final_move_X

                                elif currX < preX:

                                    newx = preX - Rpre - Rcurr + final_move_X

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                oldX, oldY, oldZ = currX, currY, currZ

                                final_move_X = preX - oldX

                                final_move_Z = preZ - oldZ

                                prev_rotation_axis = "Y"



                            elif final_move_Z == 0 and final_move_X == 0:

                                if currX > preX:

                                    newx = preX + Rpre + Rcurr

                                elif currX < preX:

                                    newx = preX - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"

                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                                oldX, oldY, oldZ = currX, currY, currZ

                                final_move_X = preX - oldX

                                final_move_Z = preZ - oldZ

                                prev_rotation_axis = "Y"



                    elif preX == currX and preY == currY and preZ != currZ:

                        gear_values = [precedent_gears_info[k] for k in sorted(precedent_gears_info)]

                        skip_post_for = False

                        for gt_key, gp_info in enumerate(gear_values, start=1):

                            oldVY = gp_info['Y']

                            oldVX = gp_info['X']

                            oldVZ = gp_info['Z']

                            oldFMX = gp_info['FinalMX']

                            oldFMZ = gp_info['FinalMZ']

                            oldR = gp_info['R']

                            if gt_key <= balance - 1:

                                if currY == oldVY:

                                    if currZ != oldVZ and currX != oldVX:

                                        if abs(currX - oldVX) > abs(currZ - oldVZ):

                                            mating_orientation = "horizontally mated"

                                            if oldFMZ != 0 and oldFMX == 0:

                                                newz = currZ + oldFMZ

                                                if currX > oldVX:

                                                    newx = oldVX + oldR + Rcurr

                                                elif currX < oldVX:

                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMZ == 0 and oldFMX != 0:

                                                if currX > oldVX:

                                                    newx = oldVX + oldR + Rcurr + oldFMX

                                                elif currX < oldVX:

                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMZ != 0 and oldFMX != 0:

                                                newz = currZ + oldFMZ

                                                if currX > oldVX:

                                                    newx = oldVX + oldR + Rcurr + oldFMX

                                                elif currX < oldVX:

                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMZ == 0 and oldVX == 0:

                                                if currX > oldVX:

                                                    newx = oldVX + oldR + Rcurr

                                                elif currX < oldVX:

                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                        elif abs(currZ - oldVZ) > abs(currX - oldVX):

                                            mating_orientation = "vertically mated"

                                            if oldFMX != 0 and oldFMZ == 0:

                                                newx = currX + oldFMX

                                                if currZ > oldVZ:

                                                    newz = oldVZ + oldR + Rcurr

                                                elif currZ < oldVZ:

                                                    newz = oldVZ - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMX == 0 and oldFMZ != 0:

                                                if currZ > oldVZ:

                                                    newz = oldVZ + oldR + Rcurr + oldFMZ

                                                elif currZ < oldVZ:

                                                    newz = oldVZ - oldR - Rcurr + oldFMZ

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMZ != 0 and oldFMX != 0:

                                                newx = currX + oldFMX

                                                if currZ > oldVZ:

                                                    newz = oldVZ + oldR + Rcurr + oldFMZ

                                                elif currZ < oldVZ:

                                                    newz = oldVZ - oldR - Rcurr + oldFMZ

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMZ == 0 and oldVX == 0:

                                                if currZ > oldVZ:

                                                    newz = oldVZ + oldR + Rcurr

                                                elif currZ < oldVZ:

                                                    newz = oldVZ - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True

                                    if oldVX != currX and oldVZ == currZ:

                                        if oldFMZ != 0 and oldFMX == 0:

                                            newz = currZ + oldFMZ

                                            if currX > oldVX:

                                                newx = oldVX + oldR + Rcurr

                                            elif currX < oldVX:

                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMZ == 0 and oldFMX != 0:

                                            if currX > oldVX:

                                                newx = oldVX + oldR + Rcurr + oldFMX

                                            elif currX < oldVX:

                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMZ != 0 and oldFMX != 0:

                                            newz = currZ + oldFMZ

                                            if currX > oldVX:

                                                newx = oldVX + oldR + Rcurr + oldFMX

                                            elif currX < oldVX:

                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMZ == 0 and oldVX == 0:

                                            if currX > oldVX:

                                                newx = oldVX + oldR + Rcurr

                                            elif currX < oldVX:

                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True

                                    if oldVX == currX and oldVZ != currZ:

                                        if oldFMX != 0 and oldFMZ == 0:

                                            newx = currX + oldFMX

                                            if currZ > oldVZ:

                                                newz = oldVZ + oldR + Rcurr

                                            elif currZ < oldVZ:

                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMX == 0 and oldFMZ != 0:

                                            if currZ > oldVZ:

                                                newz = oldVZ + oldR + Rcurr + oldFMZ

                                            elif currZ < oldVZ:

                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMZ != 0 and oldFMX != 0:

                                            newx = currX + oldFMX

                                            if currZ > oldVZ:

                                                newz = oldVZ + oldR + Rcurr + oldFMZ

                                            elif currZ < oldVZ:

                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMZ == 0 and oldVX == 0:

                                            if currZ > oldVZ:

                                                newz = oldVZ + oldR + Rcurr

                                            elif currZ < oldVZ:

                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True

                        if not skip_post_for:

                            if final_move_X != 0 and final_move_Z == 0:

                                newx = currX + final_move_X

                                if currZ > preZ:

                                    newz = preZ + Rpre + Rcurr

                                elif currZ < preZ:

                                    newz = preZ - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                oldX, oldY, oldZ = currX, currY, currZ

                                final_move_X = preX - oldX

                                final_move_Z = preZ - oldZ

                                prev_rotation_axis = "Y"


                            elif final_move_X == 0 and final_move_Z != 0:

                                if currZ > preZ:

                                    newz = preZ + Rpre + Rcurr + final_move_Z

                                elif currZ < preZ:

                                    newz = preZ - Rpre - Rcurr + final_move_Z

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                                oldX, oldY, oldZ = currX, currY, currZ

                                final_move_X = preX - oldX

                                final_move_Z = preZ - oldZ

                                prev_rotation_axis = "Y"


                            elif final_move_Z != 0 and final_move_X != 0:

                                newx = currX + final_move_X

                                if currZ > preZ:

                                    newz = preZ + Rpre + Rcurr + final_move_Z

                                elif currZ < preZ:

                                    newz = preZ - Rpre - Rcurr + final_move_Z

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                oldX, oldY, oldZ = currX, currY, currZ

                                final_move_X = preX - oldX

                                final_move_Z = preZ - oldZ

                                prev_rotation_axis = "Y"


                            elif final_move_Z == 0 and final_move_X == 0:

                                if currZ > preZ:

                                    newz = preZ + Rpre + Rcurr

                                elif currZ < preZ:

                                    newz = preZ - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                                oldX, oldY, oldZ = currX, currY, currZ

                                final_move_X = preX - oldX

                                final_move_Z = preZ - oldZ

                                prev_rotation_axis = "Y"


                    if preX != currX and preY == currY and preZ != currZ:  # refltects a shift in one of the coordinates or it just have two different ones

                        gear_values = [precedent_gears_info[k] for k in sorted(precedent_gears_info)]

                        skip_post_for = False

                        for gt_key, gp_info in enumerate(gear_values, start=1):

                            oldVY = gp_info['Y']

                            oldVX = gp_info['X']

                            oldVZ = gp_info['Z']

                            oldFMX = gp_info['FinalMX']

                            oldFMZ = gp_info['FinalMZ']

                            oldR = gp_info['R']

                            if gt_key <= balance - 1:

                                if currY == oldVY:

                                    if currZ != oldVZ and currX != oldVX:

                                        if abs(currX - oldVX) > abs(currZ - oldVZ):

                                            mating_orientation = "horizontally mated"

                                            if oldFMZ != 0 and oldFMX == 0:

                                                newz = currZ + oldFMZ

                                                if currX > oldVX:

                                                    newx = oldVX + oldR + Rcurr

                                                elif currX < oldVX:

                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMZ == 0 and oldFMX != 0:

                                                if currX > oldVX:

                                                    newx = oldVX + oldR + Rcurr + oldFMX

                                                elif currX < oldVX:

                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMZ != 0 and oldFMX != 0:

                                                newz = currZ + oldFMZ

                                                if currX > oldVX:

                                                    newx = oldVX + oldR + Rcurr + oldFMX

                                                elif currX < oldVX:

                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMZ == 0 and oldVX == 0:

                                                if currX > oldVX:

                                                    newx = oldVX + oldR + Rcurr

                                                elif currX < oldVX:

                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                        elif abs(currZ - oldVZ) > abs(currX - oldVX):

                                            mating_orientation = "vertically mated"

                                            if oldFMX != 0 and oldFMZ == 0:

                                                newx = currX + oldFMX

                                                if currZ > oldVZ:

                                                    newz = oldVZ + oldR + Rcurr

                                                elif currZ < oldVZ:

                                                    newz = oldVZ - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMX == 0 and oldFMZ != 0:

                                                if currZ > oldVZ:

                                                    newz = oldVZ + oldR + Rcurr + oldFMZ

                                                elif currZ < oldVZ:

                                                    newz = oldVZ - oldR - Rcurr + oldFMZ

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMZ != 0 and oldFMX != 0:

                                                newx = currX + oldFMX

                                                if currZ > oldVZ:

                                                    newz = oldVZ + oldR + Rcurr + oldFMZ

                                                elif currZ < oldVZ:

                                                    newz = oldVZ - oldR - Rcurr + oldFMZ

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True


                                            elif oldFMZ == 0 and oldVX == 0:

                                                if currZ > oldVZ:

                                                    newz = oldVZ + oldR + Rcurr

                                                elif currZ < oldVZ:

                                                    newz = oldVZ - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                                                oldX, oldY, oldZ = currX, currY, currZ

                                                final_move_X = preX - oldX

                                                final_move_Z = preZ - oldZ

                                                prev_rotation_axis = "Y"

                                                skip_post_for = True

                                    if oldVX != currX and oldVZ == currZ:

                                        if oldFMZ != 0 and oldFMX == 0:

                                            newz = currZ + oldFMZ

                                            if currX > oldVX:

                                                newx = oldVX + oldR + Rcurr

                                            elif currX < oldVX:

                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMZ == 0 and oldFMX != 0:

                                            if currX > oldVX:

                                                newx = oldVX + oldR + Rcurr + oldFMX

                                            elif currX < oldVX:

                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMZ != 0 and oldFMX != 0:

                                            newz = currZ + oldFMZ

                                            if currX > oldVX:

                                                newx = oldVX + oldR + Rcurr + oldFMX

                                            elif currX < oldVX:

                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMZ == 0 and oldVX == 0:

                                            if currX > oldVX:

                                                newx = oldVX + oldR + Rcurr

                                            elif currX < oldVX:

                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True

                                    if oldVX == currX and oldVZ != currZ:

                                        if oldFMX != 0 and oldFMZ == 0:

                                            newx = currX + oldFMX

                                            if currZ > oldVZ:

                                                newz = oldVZ + oldR + Rcurr

                                            elif currZ < oldVZ:

                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMX == 0 and oldFMZ != 0:

                                            if currZ > oldVZ:

                                                newz = oldVZ + oldR + Rcurr + oldFMZ

                                            elif currZ < oldVZ:

                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMZ != 0 and oldFMX != 0:

                                            newx = currX + oldFMX

                                            if currZ > oldVZ:

                                                newz = oldVZ + oldR + Rcurr + oldFMZ

                                            elif currZ < oldVZ:

                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True


                                        elif oldFMZ == 0 and oldVX == 0:

                                            if currZ > oldVZ:

                                                newz = oldVZ + oldR + Rcurr

                                            elif currZ < oldVZ:

                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                                            oldX, oldY, oldZ = currX, currY, currZ

                                            final_move_X = preX - oldX

                                            final_move_Z = preZ - oldZ

                                            prev_rotation_axis = "Y"

                                            skip_post_for = True

                        if not skip_post_for:

                            if abs(currX - preX) > abs(currZ - preZ):

                                mating_orientation = "horizontally mated"

                                if final_move_Z != 0 and final_move_X == 0:

                                    newz = currZ + final_move_Z

                                    if currX > preX:

                                        newx = preX + Rpre + Rcurr

                                    elif currX < preX:

                                        newx = preX - Rpre - Rcurr

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                    preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                    oldX, oldY, oldZ = currX, currY, currZ

                                    final_move_X = preX - oldX

                                    final_move_Z = preZ - oldZ


                                elif final_move_Z == 0 and final_move_X != 0:

                                    if currX > preX:

                                        newx = preX + Rpre + Rcurr + final_move_X

                                    elif currX < preX:

                                        newx = preX - Rpre - Rcurr + final_move_X

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"

                                    preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                                    oldX, oldY, oldZ = currX, currY, currZ

                                    final_move_X = preX - oldX

                                    final_move_Z = preZ - oldZ


                                elif final_move_Z != 0 and final_move_X != 0:

                                    newz = currZ + final_move_Z

                                    if currX > preX:

                                        newx = preX + Rpre + Rcurr + final_move_X

                                    elif currX < preX:

                                        newx = preX - Rpre - Rcurr + final_move_X

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                    preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                    oldX, oldY, oldZ = currX, currY, currZ

                                    final_move_X = preX - oldX

                                    final_move_Z = preZ - oldZ


                                elif final_move_Z == 0 and final_move_X == 0:

                                    if currX > preX:

                                        newx = preX + Rpre + Rcurr

                                    elif currX < preX:

                                        newx = preX - Rpre - Rcurr

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"

                                    preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                                    oldX, oldY, oldZ = currX, currY, currZ

                                    final_move_X = preX - oldX

                                    final_move_Z = preZ - oldZ


                            elif abs(currZ - oldZ) > abs(currX - oldX):

                                mating_orientation = "vertically mated"

                                if final_move_X != 0 and final_move_Z == 0:

                                    newx = currX + final_move_X

                                    if currZ > preZ:

                                        newz = preZ + Rpre + Rcurr

                                    elif currZ < preZ:

                                        newz = preZ - Rpre - Rcurr

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                    preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                    oldX, oldY, oldZ = currX, currY, currZ

                                    final_move_X = preX - oldX

                                    final_move_Z = preZ - oldZ


                                elif final_move_X == 0 and final_move_Z != 0:

                                    if currZ > preZ:

                                        newz = prez + Rpre + Rcurr + final_move_Z

                                    elif currZ < preZ:

                                        newz = preZ - Rpre - Rcurr + final_move_Z

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                    preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                                    oldX, oldY, oldZ = currX, currY, currZ

                                    final_move_X = preX - oldX

                                    final_move_Z = preZ - oldZ


                                elif final_move_X != 0 and final_move_Z != 0:

                                    newx = currX + final_move_X

                                    if currZ > preZ:

                                        newz = preZ + Rpre + Rcurr + final_move_Z

                                    elif currZ < preZ:

                                        newz = preZ - Rpre - Rcurr + final_move_Z

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"

                                    preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                                    oldX, oldY, oldZ = currX, currY, currZ

                                    final_move_X = preX - oldX

                                    final_move_Z = preZ - oldZ


                                elif final_move_X == 0 and final_move_Z == 0:

                                    if currZ > preZ:

                                        newz = prez + Rpre + Rcurr

                                    elif currZ < preZ:

                                        newz = preZ - Rpre - Rcurr

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                    preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                                    oldX, oldY, oldZ = currX, currY, currZ

                                    final_move_X = preX - oldX

                                    final_move_Z = preZ - oldZ



                    # Case#2 rotation around z(y in modelica)
                if eighth_rotation == 1 or eighth_rotation == -1:
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{0, {default_rotation_axis_y_tox}, 0}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{0, {gear_thick}, 0}});\n"

                    # Non-Mating case
                    if prev_rotation_axis == "Z" and preZ != currZ:
                        if preX == currX and preY == currY:
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, currZ, Rcurr
                            prev_rotation_axis = "Z"

                        if preX != currX:
                            mated_shift_indicationX = currX
                            mated_shift_indicationZ = currZ
                            mated_shift_indicationY = currY
                            newx = currX + correction_shiftx
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                            correction_shiftx = newx - currX
                            prev_rotation_axis = "Z"

                        if preY != currY:
                            mated_shift_indicationY = currY
                            mated_shift_indicationZ = currZ
                            mated_shift_indicationX = currX
                            newy = currY + correction_shifty
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                            correction_shifty = newy - currY
                            prev_rotation_axis = "Z"

                    # Mating Gear
                    if preX != currX and preY == currY and preZ == currZ:

                        if currX > preX:
                            newx = preX + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                            correction_shiftx = newx - currX
                            prev_rotation_axis = "Z"

                        if currX < preX:
                            newx = preX - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                            correction_shiftx = newx - currX
                            prev_rotation_axis = "Z"


                    if preX == currX and preY != currY and preZ == currZ:

                        if currY > preY:
                            newy = preY + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                            correction_shifty = newy - currY
                            prev_rotation_axis = "Z"

                        if currY < preY:
                            newy = preY - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                            correction_shifty = newy - currY
                            prev_rotation_axis = "Z"


                    elif preX != currX and preY != currY and preZ == currZ:
                        if currY > preY and currX < preX:
                            if abs(mated_shift_indicationY - preY) == 0:
                                newy = preY + Rpre + Rcurr
                            if abs(mated_shift_indicationY - preY) != 0:
                                newy = currY + correction_shifty  # 4cases because the imposed shift or the resultant correction shift could be in any of the values of x,y
                            if abs(mated_shift_indicationX - preX) == 0:
                                newx = preX - Rpre - Rcurr
                            if abs(mated_shift_indicationX - preX) != 0:
                                newx = currX + correction_shiftx

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, newy, currZ, Rcurr
                            correction_shifty = newy - currY
                            correction_shiftx = newx - currX
                            prev_rotation_axis = "Z"

                        if currY > preY and currX > preX:
                            if abs(mated_shift_indicationY - preY) == 0:
                                newy = preY + Rpre + Rcurr
                            if abs(mated_shift_indicationY - preY) != 0:
                                newy = currY + correction_shifty  # 4cases because the imposed shift or the resultant correction shift could be in any of the values of x,y
                            if abs(mated_shift_indicationX - preX) == 0:
                                newx = preX + Rpre + Rcurr
                            if abs(mated_shift_indicationX - preX) != 0:
                                newx = currX + correction_shiftx

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, newy, currZ, Rcurr
                            correction_shifty = newy - currY
                            correction_shiftx = newx - currX
                            prev_rotation_axis = "Z"

                        if currY < preY and currX > preX:
                            if abs(mated_shift_indicationY - preY) == 0:
                                newy = preY - Rpre - Rcurr
                            if abs(mated_shift_indicationY - preY) != 0:
                                newy = currY + correction_shifty  # 4cases because the imposed shift or the resultant correction shift could be in any of the values of x,y
                            if abs(mated_shift_indicationX - preX) == 0:
                                newx = preX + Rpre + Rcurr
                            if abs(mated_shift_indicationX - preX) != 0:
                                newx = currX + correction_shiftx

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, newy, currZ, Rcurr
                            correction_shifty = newy - currY
                            correction_shiftx = newx - currX
                            prev_rotation_axis = "Z"

                        if currY < preY and currX < preX:
                            if abs(mated_shift_indicationY - preY) == 0:
                                newy = preY - Rpre - Rcurr
                            if abs(mated_shift_indicationY - preY) != 0:
                                newy = currY + correction_shifty  # 4cases because the imposed shift or the resultant correction shift could be in any of the values of x,y
                            if abs(mated_shift_indicationX - preX) == 0:
                                newx = preX - Rpre - Rcurr
                            if abs(mated_shift_indicationX - preX) != 0:
                                newx = currX + correction_shiftx

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, newy, currZ, Rcurr
                            correction_shifty = newy - currY
                            correction_shiftx = newx - currX
                            prev_rotation_axis = "Z"

                    # Case#3 rotation around x(z in modelica)
                elif seventh_rotation == 1 or seventh_rotation == -1:
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{0, 0, {default_rotation_axis_y_tox}}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{0, {gear_thick}, 0}});\n"

                    # Non-Mating case
                    if prev_rotation_axis == "X" and preX != currX:
                        if preZ == currZ and preY == currY:
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, currZ, Rcurr
                            prev_rotation_axis = "X"

                        if preX != currY:
                            mated_shift_indicationX = currX
                            mated_shift_indicationY = currY
                            mated_shift_indicationZ = currZ
                            newy = currY + correction_shifty
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                            correction_shifty = newy - currY
                            prev_rotation_axis = "X"

                        if preZ != currZ:
                            mated_shift_indicationX = currX
                            mated_shift_indicationY = currY
                            mated_shift_indicationZ = currZ
                            newz = currZ + correction_shiftz
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                            correction_shiftz = newz - currZ
                            prev_rotation_axis = "X"

                    # Mating Gear
                    if preX == currX and preY != currY and preZ == currZ:

                        if currY > preY:
                            newy = preY + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                            correction_shifty = newy - currY
                            prev_rotation_axis = "X"
                        if currY < preY:
                            newy = preX - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                            correction_shifty = newy - currY
                            prev_rotation_axis = "X"

                    if preX == currX and preY == currY and preZ != currZ:

                        if currZ > preZ:
                            newz = preZ + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                            correction_shiftz = newz - currZ
                            prev_rotation_axis = "X"
                        if currZ < preZ:
                            newz = preX - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                            correction_shiftz = newz - currZ
                            prev_rotation_axis = "X"


                    elif preX == currX and preY != currY and preZ != currZ:
                        if currZ > preZ and currY < preY:
                            if abs(mated_shift_indicationY - preY) == 0:
                                newy = preY - Rpre - Rcurr
                            if abs(mated_shift_indicationY - preY) != 0:
                                newy = currY + correction_shifty  # 4cases because the imposed shift or the resultant correction shift could be in any of the values of z,y
                            if abs(mated_shift_indicationZ - preZ) == 0:
                                newz = preZ + Rpre + Rcurr
                            if abs(mated_shift_indicationZ - preZ) != 0:
                                newz = currZ + correction_shiftz

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, newz, Rcurr
                            correction_shiftz = newz - currZ
                            correction_shifty = newy - currY
                            prev_rotation_axis = "X"

                        if currZ > preZ and currY > preY:
                            if abs(mated_shift_indicationY - preY) == 0:
                                newy = preY + Rpre + Rcurr
                            if abs(mated_shift_indicationY - preY) != 0:
                                newy = currY + correction_shifty  # 4cases because the imposed shift or the resultant correction shift could be in any of the values of z,y
                            if abs(mated_shift_indicationZ - preZ) == 0:
                                newz = preZ + Rpre + Rcurr
                            if abs(mated_shift_indicationZ - preZ) != 0:
                                newz = currZ + correction_shiftz

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, newz, Rcurr
                            correction_shiftz = newz - currZ
                            correction_shifty = newy - currY
                            prev_rotation_axis = "X"

                        if currZ < preZ and currY > preY:
                            if abs(mated_shift_indicationY - preY) == 0:
                                newy = preY + Rpre + Rcurr
                            if abs(mated_shift_indicationY - preY) != 0:
                                newy = currY + correction_shifty  # 4cases because the imposed shift or the resultant correction shift could be in any of the values of z,y
                            if abs(mated_shift_indicationZ - preZ) == 0:
                                newz = preZ - Rpre - Rcurr
                            if abs(mated_shift_indicationZ - preZ) != 0:
                                newz = currZ + correction_shiftz

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, newz, Rcurr
                            correction_shiftz = newz - currZ
                            correction_shifty = newy - currY
                            prev_rotation_axis = "X"

                        if currZ < preZ and currY < preY:
                            if abs(mated_shift_indicationY - preY) == 0:
                                newy = preY - Rpre - Rcurr
                            if abs(mated_shift_indicationY - preY) != 0:
                                newy = currY + correction_shifty  # 4cases because the imposed shift or the resultant correction shift could be in any of the values of z,y
                            if abs(mated_shift_indicationZ - preZ) == 0:
                                newz = preZ - Rpre - Rcurr
                            if abs(mated_shift_indicationZ - preZ) != 0:
                                newz = currZ + correction_shiftz

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, newz, Rcurr
                            correction_shiftz = newz - currZ
                            correction_shifty = newy - currY
                            prev_rotation_axis = "X"


            if gear_id == last_idx:
                
                modelica_code_part += f"  Modelica.Mechanics.Rotational.Components.IdealGear idealGear{balance}(ratio = {gear_ratio});\n"

                modelica_code_connection += f"  connect({body_position_name}.frame_a, world.frame_b);\n"
                modelica_code_connection += f"  connect({body_position_name}.frame_b, {actuate_revolute_name}.frame_a);\n"
                modelica_code_connection += f"  connect({actuate_revolute_name}.frame_b, {body_name}.frame_a);\n"
                modelica_code_connection += f"  connect(idealGear{balance}.flange_a, actuatedRevolute_{balance}.axis);\n"
                modelica_code_connection += f"  connect(idealGear{balance}.flange_b, {actuate_revolute_name}.axis);\n"

                #Case#1 default_orientation (the gear rotates around Y-Axis in LeoCAD & X-Axis in OM)
                if ninth_rotation == 1 or ninth_rotation == -1:
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{{default_rotation_axis_y_tox}, 0, 0}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{{gear_thick}, 0, 0}});\n"

                    # Non-Mating case
                    if prev_rotation_axis == "Y" and preY != currY:
                        if preX == currX and preZ == currZ:
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, currZ, Rcurr
                            prev_rotation_axis = "Y"

                        elif preX != currX:
                            newx = currX + final_move_X
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                            oldX, oldY, oldZ = currX, currY, currZ
                            final_move_X = preX - oldX
                            final_move_Z = preZ - oldZ
                            prev_rotation_axis = "Y"


                        elif preZ != currZ:
                            newz = currZ + final_move_Z
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                            oldX, oldY, oldZ = currX, currY, currZ
                            final_move_X = preX - oldX
                            final_move_Z = preZ - oldZ
                            prev_rotation_axis = "Y"

                    #Mating Gear
                    elif preX != currX and preY == currY and preZ == currZ:

                        gear_values = [precedent_gears_info[k] for k in sorted(precedent_gears_info)]

                        skip_post_for = False

                        for gt_key, gp_info in enumerate(gear_values, start=1):

                            oldVY = gp_info['Y']
                            oldVX = gp_info['X']
                            oldVZ = gp_info['Z']
                            oldFMX = gp_info['FinalMX']
                            oldFMZ = gp_info['FinalMZ']
                            oldR = gp_info['R']

                            if gt_key <= balance - 1:

                                if currY == oldVY:

                                    if currZ != oldVZ and currX != oldVX:

                                        if abs(currX - oldVX) > abs(currZ - oldVZ):
                                            mating_orientation = "horizontally mated"

                                            if oldFMZ != 0 and oldFMX == 0:
                                                newz = currZ + oldFMZ
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ == 0 and oldFMX != 0:
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr + oldFMX
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ != 0 and oldFMX != 0:
                                                newz = currZ + oldFMZ
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr + oldFMX
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ == 0 and oldVX == 0:
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                        elif abs(currZ - oldVZ) > abs(currX - oldVX):
                                            mating_orientation = "vertically mated"

                                            if oldFMX != 0 and oldFMZ == 0:
                                                newx = currX + oldFMX
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMX == 0 and oldFMZ != 0:
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr + oldFMZ
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr + oldFMZ

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ != 0 and oldFMX != 0:
                                                newx = currX + oldFMX
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr + oldFMZ
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr + oldFMZ

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ == 0 and oldVX == 0:
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                    if oldVX != currX and oldVZ == currZ:

                                        if oldFMZ != 0 and oldFMX == 0:
                                            newz = currZ + oldFMZ
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldFMX != 0:
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr + oldFMX
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ != 0 and oldFMX != 0:
                                            newz = currZ + oldFMZ
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr + oldFMX
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldVX == 0:
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                    if oldVX == currX and oldVZ != currZ:

                                        if oldFMX != 0 and oldFMZ == 0:
                                            newx = currX + oldFMX
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMX == 0 and oldFMZ != 0:
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr + oldFMZ
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ != 0 and oldFMX != 0:
                                            newx = currX + oldFMX
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr + oldFMZ
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldVX == 0:
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                        if not skip_post_for:

                            if final_move_Z != 0 and final_move_X == 0:
                                newz = currZ + final_move_Z
                                if currX > preX:
                                    newx = preX + Rpre + Rcurr
                                elif currX < preX:
                                    newx = preX - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                oldX, oldY, oldZ = currX, currY, currZ
                                final_move_X = preX - oldX
                                final_move_Z = preZ - oldZ
                                prev_rotation_axis = "Y"


                            elif final_move_Z == 0 and final_move_X != 0:
                                if currX > preX:
                                    newx = preX + Rpre + Rcurr + final_move_X
                                elif currX < preX:
                                    newx = preX - Rpre - Rcurr + final_move_X

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                oldX, oldY, oldZ = currX, currY, currZ
                                final_move_X = preX - oldX
                                final_move_Z = preZ - oldZ
                                prev_rotation_axis = "Y"

                            elif final_move_Z != 0 and final_move_X != 0:
                                newz = currZ + final_move_Z
                                if currX > preX:
                                    newx = preX + Rpre + Rcurr + final_move_X
                                elif currX < preX:
                                    newx = preX - Rpre - Rcurr + final_move_X

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                oldX, oldY, oldZ = currX, currY, currZ
                                final_move_X = preX - oldX
                                final_move_Z = preZ - oldZ
                                prev_rotation_axis = "Y"

                            elif final_move_Z == 0 and final_move_X == 0:
                                if currX > preX:
                                    newx = preX + Rpre + Rcurr
                                elif currX < preX:
                                    newx = preX - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                oldX, oldY, oldZ = currX, currY, currZ
                                final_move_X = preX - oldX
                                final_move_Z = preZ - oldZ
                                prev_rotation_axis = "Y"

                    elif preX == currX and preY == currY and preZ != currZ:

                        gear_values = [precedent_gears_info[k] for k in sorted(precedent_gears_info)]

                        skip_post_for = False

                        for gt_key, gp_info in enumerate(gear_values, start=1):

                            oldVY = gp_info['Y']
                            oldVX = gp_info['X']
                            oldVZ = gp_info['Z']
                            oldFMX = gp_info['FinalMX']
                            oldFMZ = gp_info['FinalMZ']
                            oldR = gp_info['R']

                            if gt_key <= balance - 1:

                                if currY == oldVY:

                                    if currZ != oldVZ and currX != oldVX:

                                        if abs(currX - oldVX) > abs(currZ - oldVZ):
                                            mating_orientation = "horizontally mated"

                                            if oldFMZ != 0 and oldFMX == 0:
                                                newz = currZ + oldFMZ
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ == 0 and oldFMX != 0:
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr + oldFMX
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ != 0 and oldFMX != 0:
                                                newz = currZ + oldFMZ
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr + oldFMX
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ == 0 and oldVX == 0:
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                        elif abs(currZ - oldVZ) > abs(currX - oldVX):
                                            mating_orientation = "vertically mated"

                                            if oldFMX != 0 and oldFMZ == 0:
                                                newx = currX + oldFMX
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMX == 0 and oldFMZ != 0:
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr + oldFMZ
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr + oldFMZ

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ != 0 and oldFMX != 0:
                                                newx = currX + oldFMX
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr + oldFMZ
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr + oldFMZ

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ == 0 and oldVX == 0:
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                    if oldVX != currX and oldVZ == currZ:

                                        if oldFMZ != 0 and oldFMX == 0:
                                            newz = currZ + oldFMZ
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldFMX != 0:
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr + oldFMX
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ != 0 and oldFMX != 0:
                                            newz = currZ + oldFMZ
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr + oldFMX
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldVX == 0:
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                    if oldVX == currX and oldVZ != currZ:

                                        if oldFMX != 0 and oldFMZ == 0:
                                            newx = currX + oldFMX
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMX == 0 and oldFMZ != 0:
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr + oldFMZ
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ != 0 and oldFMX != 0:
                                            newx = currX + oldFMX
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr + oldFMZ
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldVX == 0:
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                        if not skip_post_for:

                            if final_move_X != 0 and final_move_Z == 0:
                                newx = currX + final_move_X
                                if currZ > preZ:
                                    newz = preZ + Rpre + Rcurr
                                elif currZ < preZ:
                                    newz = preZ - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                oldX, oldY, oldZ = currX, currY, currZ
                                final_move_X = preX - oldX
                                final_move_Z = preZ - oldZ
                                prev_rotation_axis = "Y"

                            elif final_move_X == 0 and final_move_Z != 0:
                                if currZ > preZ:
                                    newz = preZ + Rpre + Rcurr + final_move_Z
                                elif currZ < preZ:
                                    newz = preZ - Rpre - Rcurr + final_move_Z

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                oldX, oldY, oldZ = currX, currY, currZ
                                final_move_X = preX - oldX
                                final_move_Z = preZ - oldZ
                                prev_rotation_axis = "Y"


                            elif final_move_Z != 0 and final_move_X != 0:
                                newx = currX + final_move_X
                                if currZ > preZ:
                                    newz = preZ + Rpre + Rcurr + final_move_Z
                                elif currZ < preZ:
                                    newz = preZ - Rpre - Rcurr + final_move_Z

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                oldX, oldY, oldZ = currX, currY, currZ
                                final_move_X = preX - oldX
                                final_move_Z = preZ - oldZ
                                prev_rotation_axis = "Y"

                            elif final_move_Z == 0 and final_move_X == 0:
                                if currZ > preZ:
                                    newz = preZ + Rpre + Rcurr
                                elif currZ < preZ:
                                    newz = preZ - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                oldX, oldY, oldZ = currX, currY, currZ
                                final_move_X = preX - oldX
                                final_move_Z = preZ - oldZ
                                prev_rotation_axis = "Y"


                    if preX != currX and preY == currY and preZ != currZ:     # refltects a shift in one of the coordinates or it just have two different ones
                        gear_values = [precedent_gears_info[k] for k in sorted(precedent_gears_info)]

                        skip_post_for = False

                        for gt_key, gp_info  in enumerate (gear_values, start=1):


                            oldVY = gp_info['Y']
                            oldVX = gp_info['X']
                            oldVZ = gp_info['Z']
                            oldFMX = gp_info['FinalMX']
                            oldFMZ = gp_info['FinalMZ']
                            oldR = gp_info['R']



                            if gt_key <= balance-1:

                                if currY == oldVY:

                                    if currZ != oldVZ and currX != oldVX:

                                        if abs(currX - oldVX) > abs(currZ - oldVZ):
                                            mating_orientation = "horizontally mated"

                                            if oldFMZ != 0 and oldFMX == 0:
                                                newz = currZ + oldFMZ
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ == 0 and oldFMX != 0:
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr + oldFMX
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ != 0 and oldFMX != 0:
                                                newz = currZ + oldFMZ
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr + oldFMX
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ == 0 and oldVX == 0:
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                        elif abs(currZ - oldVZ) > abs(currX - oldVX):
                                            mating_orientation = "vertically mated"

                                            if oldFMX != 0 and oldFMZ == 0:
                                                newx = currX + oldFMX
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMX == 0 and oldFMZ != 0:
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr + oldFMZ
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr + oldFMZ

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ != 0 and oldFMX != 0:
                                                newx = currX + oldFMX
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr + oldFMZ
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr + oldFMZ

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ == 0 and oldVX == 0:
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True


                                    if oldVX != currX and oldVZ == currZ:

                                        if oldFMZ != 0 and oldFMX == 0:
                                            newz = currZ + oldFMZ
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldFMX != 0:
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr + oldFMX
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ != 0 and oldFMX != 0:
                                            newz = currZ + oldFMZ
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr + oldFMX
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldVX == 0:
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True


                                    if oldVX == currX and oldVZ != currZ:

                                        if oldFMX != 0 and oldFMZ == 0:
                                            newx = currX + oldFMX
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMX == 0 and oldFMZ != 0:
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr + oldFMZ
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ != 0 and oldFMX != 0:
                                            newx = currX + oldFMX
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr + oldFMZ
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldVX == 0:
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True
                                            
                        if not skip_post_for:
                            if abs(currX - preX) > abs(currZ - preZ):
                                mating_orientation = "horizontally mated"

                                if final_move_Z != 0 and final_move_X == 0:
                                    newz = currZ + final_move_Z
                                    if currX > preX:
                                        newx = preX + Rpre + Rcurr
                                    elif currX < preX:
                                        newx = preX - Rpre - Rcurr

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                    preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                    oldX, oldY, oldZ = currX, currY, currZ
                                    final_move_X = preX - oldX
                                    final_move_Z = preZ - oldZ

                                elif final_move_Z == 0 and final_move_X != 0:
                                    if currX > preX:
                                        newx = preX + Rpre + Rcurr + final_move_X
                                    elif currX < preX:
                                        newx = preX - Rpre - Rcurr + final_move_X

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                    preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                    oldX, oldY, oldZ = currX, currY, currZ
                                    final_move_X = preX - oldX
                                    final_move_Z = preZ - oldZ

                                elif final_move_Z != 0 and final_move_X != 0:
                                    newz = currZ + final_move_Z
                                    if currX > preX:
                                        newx = preX + Rpre + Rcurr + final_move_X
                                    elif currX < preX:
                                        newx = preX - Rpre - Rcurr + final_move_X

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                    preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                    oldX, oldY, oldZ = currX, currY, currZ
                                    final_move_X = preX - oldX
                                    final_move_Z = preZ - oldZ

                                elif final_move_Z == 0 and final_move_X == 0:
                                    if currX > preX:
                                        newx = preX + Rpre + Rcurr
                                    elif currX < preX:
                                        newx = preX - Rpre - Rcurr

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                    preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                    oldX, oldY, oldZ = currX, currY, currZ
                                    final_move_X = preX - oldX
                                    final_move_Z = preZ - oldZ

                            elif abs(currZ - oldZ) > abs(currX - oldX):
                                mating_orientation = "vertically mated"

                                if final_move_X != 0 and final_move_Z == 0:
                                    newx = currX + final_move_X
                                    if currZ > preZ:
                                        newz = preZ + Rpre + Rcurr
                                    elif currZ < preZ:
                                        newz = preZ - Rpre - Rcurr

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                    preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                    oldX, oldY, oldZ = currX, currY, currZ
                                    final_move_X = preX - oldX
                                    final_move_Z = preZ - oldZ

                                elif final_move_X == 0 and final_move_Z != 0:
                                    if currZ > preZ:
                                        newz = prez + Rpre + Rcurr + final_move_Z
                                    elif currZ < preZ:
                                        newz = preZ - Rpre - Rcurr + final_move_Z

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                    preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                    oldX, oldY, oldZ = currX, currY, currZ
                                    final_move_X = preX - oldX
                                    final_move_Z = preZ - oldZ

                                elif final_move_X != 0 and final_move_Z != 0:
                                    newx = currX + final_move_X
                                    if currZ > preZ:
                                        newz = preZ + Rpre + Rcurr + final_move_Z
                                    elif currZ < preZ:
                                        newz = preZ - Rpre - Rcurr + final_move_Z

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                    preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                    oldX, oldY, oldZ = currX, currY, currZ
                                    final_move_X = preX - oldX
                                    final_move_Z = preZ - oldZ

                                elif final_move_X == 0 and final_move_Z == 0:
                                    if currZ > preZ:
                                        newz = prez + Rpre + Rcurr
                                    elif currZ < preZ:
                                        newz = preZ - Rpre - Rcurr

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                    preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                    oldX, oldY, oldZ = currX, currY, currZ
                                    final_move_X = preX - oldX
                                    final_move_Z = preZ - oldZ




                    # Case#2 rotation around z(y in modelica)
                if eighth_rotation == 1 or eighth_rotation == -1 :
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{0, {default_rotation_axis_y_tox}, 0}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{0, 0, {gear_thick}}});\n"

                    # Non-Mating case
                    if prev_rotation_axis == "Z" and preZ != currZ:
                        if preX == currX and preY == currY:
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, currZ, Rcurr
                            prev_rotation_axis = "Z"

                        elif preX != currX:
                            newx = currX + final_move_X
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                            oldX, oldY, oldZ = currX, currY, currZ
                            final_move_X = preX - oldX
                            final_move_Y = preY - oldY
                            prev_rotation_axis = "Z"


                        elif preY != currY:
                            newy = currY + final_move_Y
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                            oldX, oldY, oldZ = currX, currY, currZ
                            final_move_X = preX - oldX
                            final_move_Y = preY - oldY
                            prev_rotation_axis = "Z"

                    # Mating Gear
                    elif preX != currX and preY == currY and preZ == currZ:

                        gear_values = [precedent_gears_info[k] for k in sorted(precedent_gears_info)]

                        skip_post_for = False

                        for gt_key, gp_info in enumerate(gear_values, start=1):

                            oldVY = gp_info['Y']
                            oldVX = gp_info['X']
                            oldVZ = gp_info['Z']
                            oldFMX = gp_info['FinalMX']
                            oldFMY = gp_info['FinalMY']
                            oldR = gp_info['R']

                            if gt_key <= balance - 1:

                                if currZ == oldVZ:

                                    if currY != oldVY and currX != oldVX:

                                        if abs(currX - oldVX) > abs(currY - oldVY):
                                            mating_orientation = "vertically mated"

                                            if oldFMY != 0 and oldFMX == 0:
                                                newy = currY + oldFMY
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, newy, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Y = preY - oldY
                                                prev_rotation_axis = "Z"
                                                skip_post_for = True

                                            elif oldFMY == 0 and oldFMX != 0:
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr + oldFMX
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Y = preY - oldY
                                                prev_rotation_axis = "Z"
                                                skip_post_for = True

                                            elif oldFMY != 0 and oldFMX != 0:
                                                newy = currY + oldFMY
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr + oldFMX
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, newy, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preY - oldY
                                                prev_rotation_axis = "Z"
                                                skip_post_for = True

                                            elif oldFMY == 0 and oldVX == 0:
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Y = preY - oldY
                                                prev_rotation_axis = "Z"
                                                skip_post_for = True

                                        elif abs(currY - oldVY) > abs(currX - oldVX):
                                            mating_orientation = "horizontally mated"

                                            if oldFMX != 0 and oldFMY == 0:
                                                newx = currX + oldFMX
                                                if currY > oldVY:
                                                    newz = oldVY + oldR + Rcurr
                                                elif currY < oldVY:
                                                    newy = oldVY - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, newy, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preY - oldY
                                                prev_rotation_axis = "Z"
                                                skip_post_for = True

                                            elif oldFMX == 0 and oldFMY != 0:
                                                if currY > oldVY:
                                                    newy = oldVY + oldR + Rcurr + oldFMY
                                                elif currY < oldVY:
                                                    newy = oldVY - oldR - Rcurr + oldFMY

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preY - oldY
                                                prev_rotation_axis = "Z"
                                                skip_post_for = True

                                            elif oldFMY != 0 and oldFMX != 0:
                                                newx = currX + oldFMX
                                                if currY > oldVY:
                                                    newy = oldVY + oldR + Rcurr + oldFMY
                                                elif currY < oldVY:
                                                    newy = oldVY - oldR - Rcurr + oldFMY

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, newy, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Y = preY - oldY
                                                prev_rotation_axis = "Z"
                                                skip_post_for = True

                                            elif oldFMY == 0 and oldVX == 0:
                                                if currY > oldVY:
                                                    newy = oldVY + oldR + Rcurr
                                                elif currY < oldVY:
                                                    newy = oldVY - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Y = preY - oldY
                                                prev_rotation_axis = "Z"
                                                skip_post_for = True

                                    if oldVX != currX and oldVZ == currZ:

                                        if oldFMZ != 0 and oldFMX == 0:
                                            newz = currZ + oldFMZ
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldFMX != 0:
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr + oldFMX
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ != 0 and oldFMX != 0:
                                            newz = currZ + oldFMZ
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr + oldFMX
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldVX == 0:
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                    if oldVX == currX and oldVZ != currZ:

                                        if oldFMX != 0 and oldFMZ == 0:
                                            newx = currX + oldFMX
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMX == 0 and oldFMZ != 0:
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr + oldFMZ
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ != 0 and oldFMX != 0:
                                            newx = currX + oldFMX
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr + oldFMZ
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldVX == 0:
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                        if not skip_post_for:

                            if final_move_Z != 0 and final_move_X == 0:
                                newz = currZ + final_move_Z
                                if currX > preX:
                                    newx = preX + Rpre + Rcurr
                                elif currX < preX:
                                    newx = preX - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                oldX, oldY, oldZ = currX, currY, currZ
                                final_move_X = preX - oldX
                                final_move_Z = preZ - oldZ
                                prev_rotation_axis = "Y"


                            elif final_move_Z == 0 and final_move_X != 0:
                                if currX > preX:
                                    newx = preX + Rpre + Rcurr + final_move_X
                                elif currX < preX:
                                    newx = preX - Rpre - Rcurr + final_move_X

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                oldX, oldY, oldZ = currX, currY, currZ
                                final_move_X = preX - oldX
                                final_move_Z = preZ - oldZ
                                prev_rotation_axis = "Y"

                            elif final_move_Z != 0 and final_move_X != 0:
                                newz = currZ + final_move_Z
                                if currX > preX:
                                    newx = preX + Rpre + Rcurr + final_move_X
                                elif currX < preX:
                                    newx = preX - Rpre - Rcurr + final_move_X

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                oldX, oldY, oldZ = currX, currY, currZ
                                final_move_X = preX - oldX
                                final_move_Z = preZ - oldZ
                                prev_rotation_axis = "Y"

                            elif final_move_Z == 0 and final_move_X == 0:
                                if currX > preX:
                                    newx = preX + Rpre + Rcurr
                                elif currX < preX:
                                    newx = preX - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                oldX, oldY, oldZ = currX, currY, currZ
                                final_move_X = preX - oldX
                                final_move_Z = preZ - oldZ
                                prev_rotation_axis = "Y"

                    elif preX == currX and preY == currY and preZ != currZ:

                        gear_values = [precedent_gears_info[k] for k in sorted(precedent_gears_info)]

                        skip_post_for = False

                        for gt_key, gp_info in enumerate(gear_values, start=1):

                            oldVY = gp_info['Y']
                            oldVX = gp_info['X']
                            oldVZ = gp_info['Z']
                            oldFMX = gp_info['FinalMX']
                            oldFMZ = gp_info['FinalMZ']
                            oldR = gp_info['R']

                            if gt_key <= balance - 1:

                                if currY == oldVY:

                                    if currZ != oldVZ and currX != oldVX:

                                        if abs(currX - oldVX) > abs(currZ - oldVZ):
                                            mating_orientation = "horizontally mated"

                                            if oldFMZ != 0 and oldFMX == 0:
                                                newz = currZ + oldFMZ
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ == 0 and oldFMX != 0:
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr + oldFMX
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ != 0 and oldFMX != 0:
                                                newz = currZ + oldFMZ
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr + oldFMX
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ == 0 and oldVX == 0:
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                        elif abs(currZ - oldVZ) > abs(currX - oldVX):
                                            mating_orientation = "vertically mated"

                                            if oldFMX != 0 and oldFMZ == 0:
                                                newx = currX + oldFMX
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMX == 0 and oldFMZ != 0:
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr + oldFMZ
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr + oldFMZ

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ != 0 and oldFMX != 0:
                                                newx = currX + oldFMX
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr + oldFMZ
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr + oldFMZ

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ == 0 and oldVX == 0:
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                    if oldVX != currX and oldVZ == currZ:

                                        if oldFMZ != 0 and oldFMX == 0:
                                            newz = currZ + oldFMZ
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldFMX != 0:
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr + oldFMX
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ != 0 and oldFMX != 0:
                                            newz = currZ + oldFMZ
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr + oldFMX
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldVX == 0:
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                    if oldVX == currX and oldVZ != currZ:

                                        if oldFMX != 0 and oldFMZ == 0:
                                            newx = currX + oldFMX
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMX == 0 and oldFMZ != 0:
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr + oldFMZ
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ != 0 and oldFMX != 0:
                                            newx = currX + oldFMX
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr + oldFMZ
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldVX == 0:
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                        if not skip_post_for:

                            if final_move_X != 0 and final_move_Z == 0:
                                newx = currX + final_move_X
                                if currZ > preZ:
                                    newz = preZ + Rpre + Rcurr
                                elif currZ < preZ:
                                    newz = preZ - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                oldX, oldY, oldZ = currX, currY, currZ
                                final_move_X = preX - oldX
                                final_move_Z = preZ - oldZ
                                prev_rotation_axis = "Y"

                            elif final_move_X == 0 and final_move_Z != 0:
                                if currZ > preZ:
                                    newz = preZ + Rpre + Rcurr + final_move_Z
                                elif currZ < preZ:
                                    newz = preZ - Rpre - Rcurr + final_move_Z

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                oldX, oldY, oldZ = currX, currY, currZ
                                final_move_X = preX - oldX
                                final_move_Z = preZ - oldZ
                                prev_rotation_axis = "Y"


                            elif final_move_Z != 0 and final_move_X != 0:
                                newx = currX + final_move_X
                                if currZ > preZ:
                                    newz = preZ + Rpre + Rcurr + final_move_Z
                                elif currZ < preZ:
                                    newz = preZ - Rpre - Rcurr + final_move_Z

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                oldX, oldY, oldZ = currX, currY, currZ
                                final_move_X = preX - oldX
                                final_move_Z = preZ - oldZ
                                prev_rotation_axis = "Y"

                            elif final_move_Z == 0 and final_move_X == 0:
                                if currZ > preZ:
                                    newz = preZ + Rpre + Rcurr
                                elif currZ < preZ:
                                    newz = preZ - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                oldX, oldY, oldZ = currX, currY, currZ
                                final_move_X = preX - oldX
                                final_move_Z = preZ - oldZ
                                prev_rotation_axis = "Y"

                    if preX != currX and preY == currY and preZ != currZ:  # refltects a shift in one of the coordinates or it just have two different ones
                        gear_values = [precedent_gears_info[k] for k in sorted(precedent_gears_info)]

                        skip_post_for = False

                        for gt_key, gp_info in enumerate(gear_values, start=1):

                            oldVY = gp_info['Y']
                            oldVX = gp_info['X']
                            oldVZ = gp_info['Z']
                            oldFMX = gp_info['FinalMX']
                            oldFMZ = gp_info['FinalMZ']
                            oldR = gp_info['R']

                            if gt_key <= balance - 1:

                                if currY == oldVY:

                                    if currZ != oldVZ and currX != oldVX:

                                        if abs(currX - oldVX) > abs(currZ - oldVZ):
                                            mating_orientation = "horizontally mated"

                                            if oldFMZ != 0 and oldFMX == 0:
                                                newz = currZ + oldFMZ
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ == 0 and oldFMX != 0:
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr + oldFMX
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ != 0 and oldFMX != 0:
                                                newz = currZ + oldFMZ
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr + oldFMX
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr + oldFMX

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ == 0 and oldVX == 0:
                                                if currX > oldVX:
                                                    newx = oldVX + oldR + Rcurr
                                                elif currX < oldVX:
                                                    newx = oldVX - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                        elif abs(currZ - oldVZ) > abs(currX - oldVX):
                                            mating_orientation = "vertically mated"

                                            if oldFMX != 0 and oldFMZ == 0:
                                                newx = currX + oldFMX
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMX == 0 and oldFMZ != 0:
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr + oldFMZ
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr + oldFMZ

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ != 0 and oldFMX != 0:
                                                newx = currX + oldFMX
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr + oldFMZ
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr + oldFMZ

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                                preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                            elif oldFMZ == 0 and oldVX == 0:
                                                if currZ > oldVZ:
                                                    newz = oldVZ + oldR + Rcurr
                                                elif currZ < oldVZ:
                                                    newz = oldVZ - oldR - Rcurr

                                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                                oldX, oldY, oldZ = currX, currY, currZ
                                                final_move_X = preX - oldX
                                                final_move_Z = preZ - oldZ
                                                prev_rotation_axis = "Y"
                                                skip_post_for = True

                                    if oldVX != currX and oldVZ == currZ:

                                        if oldFMZ != 0 and oldFMX == 0:
                                            newz = currZ + oldFMZ
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldFMX != 0:
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr + oldFMX
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ != 0 and oldFMX != 0:
                                            newz = currZ + oldFMZ
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr + oldFMX
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr + oldFMX

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldVX == 0:
                                            if currX > oldVX:
                                                newx = oldVX + oldR + Rcurr
                                            elif currX < oldVX:
                                                newx = oldVX - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                    if oldVX == currX and oldVZ != currZ:

                                        if oldFMX != 0 and oldFMZ == 0:
                                            newx = currX + oldFMX
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMX == 0 and oldFMZ != 0:
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr + oldFMZ
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ != 0 and oldFMX != 0:
                                            newx = currX + oldFMX
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr + oldFMZ
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr + oldFMZ

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                            preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                                        elif oldFMZ == 0 and oldVX == 0:
                                            if currZ > oldVZ:
                                                newz = oldVZ + oldR + Rcurr
                                            elif currZ < oldVZ:
                                                newz = oldVZ - oldR - Rcurr

                                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                            oldX, oldY, oldZ = currX, currY, currZ
                                            final_move_X = preX - oldX
                                            final_move_Z = preZ - oldZ
                                            prev_rotation_axis = "Y"
                                            skip_post_for = True

                        if not skip_post_for:
                            if abs(currX - preX) > abs(currZ - preZ):
                                mating_orientation = "horizontally mated"

                                if final_move_Z != 0 and final_move_X == 0:
                                    newz = currZ + final_move_Z
                                    if currX > preX:
                                        newx = preX + Rpre + Rcurr
                                    elif currX < preX:
                                        newx = preX - Rpre - Rcurr

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                    preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                    oldX, oldY, oldZ = currX, currY, currZ
                                    final_move_X = preX - oldX
                                    final_move_Z = preZ - oldZ

                                elif final_move_Z == 0 and final_move_X != 0:
                                    if currX > preX:
                                        newx = preX + Rpre + Rcurr + final_move_X
                                    elif currX < preX:
                                        newx = preX - Rpre - Rcurr + final_move_X

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                    preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                    oldX, oldY, oldZ = currX, currY, currZ
                                    final_move_X = preX - oldX
                                    final_move_Z = preZ - oldZ

                                elif final_move_Z != 0 and final_move_X != 0:
                                    newz = currZ + final_move_Z
                                    if currX > preX:
                                        newx = preX + Rpre + Rcurr + final_move_X
                                    elif currX < preX:
                                        newx = preX - Rpre - Rcurr + final_move_X

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                    preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                    oldX, oldY, oldZ = currX, currY, currZ
                                    final_move_X = preX - oldX
                                    final_move_Z = preZ - oldZ

                                elif final_move_Z == 0 and final_move_X == 0:
                                    if currX > preX:
                                        newx = preX + Rpre + Rcurr
                                    elif currX < preX:
                                        newx = preX - Rpre - Rcurr

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                    preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                                    oldX, oldY, oldZ = currX, currY, currZ
                                    final_move_X = preX - oldX
                                    final_move_Z = preZ - oldZ

                            elif abs(currZ - oldZ) > abs(currX - oldX):
                                mating_orientation = "vertically mated"

                                if final_move_X != 0 and final_move_Z == 0:
                                    newx = currX + final_move_X
                                    if currZ > preZ:
                                        newz = preZ + Rpre + Rcurr
                                    elif currZ < preZ:
                                        newz = preZ - Rpre - Rcurr

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                    preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                    oldX, oldY, oldZ = currX, currY, currZ
                                    final_move_X = preX - oldX
                                    final_move_Z = preZ - oldZ

                                elif final_move_X == 0 and final_move_Z != 0:
                                    if currZ > preZ:
                                        newz = prez + Rpre + Rcurr + final_move_Z
                                    elif currZ < preZ:
                                        newz = preZ - Rpre - Rcurr + final_move_Z

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                    preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                    oldX, oldY, oldZ = currX, currY, currZ
                                    final_move_X = preX - oldX
                                    final_move_Z = preZ - oldZ

                                elif final_move_X != 0 and final_move_Z != 0:
                                    newx = currX + final_move_X
                                    if currZ > preZ:
                                        newz = preZ + Rpre + Rcurr + final_move_Z
                                    elif currZ < preZ:
                                        newz = preZ - Rpre - Rcurr + final_move_Z

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                                    preX, preY, preZ, Rpre = newx, currY, newz, Rcurr
                                    oldX, oldY, oldZ = currX, currY, currZ
                                    final_move_X = preX - oldX
                                    final_move_Z = preZ - oldZ

                                elif final_move_X == 0 and final_move_Z == 0:
                                    if currZ > preZ:
                                        newz = prez + Rpre + Rcurr
                                    elif currZ < preZ:
                                        newz = preZ - Rpre - Rcurr

                                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                    preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                                    oldX, oldY, oldZ = currX, currY, currZ
                                    final_move_X = preX - oldX
                                    final_move_Z = preZ - oldZ

                    # Case#3 rotation around x(z in modelica)
                elif seventh_rotation == 1 or seventh_rotation == 1 :
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{0, 0, {default_rotation_axis_y_tox}}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{0, {gear_thick}, 0}});\n"

                    # Non-Mating case
                    if prev_rotation_axis == "X" and preX != currX:
                        if preZ == currZ and preY == currY:
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, currZ, Rcurr
                            prev_rotation_axis = "X"

                        if preX != currY:
                            mated_shift_indicationX = currX
                            mated_shift_indicationY = currY
                            mated_shift_indicationZ = currZ
                            newy = currY + correction_shifty
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                            correction_shifty = newy - currY
                            prev_rotation_axis = "X"

                        if preZ != currZ:
                            mated_shift_indicationX = currX
                            mated_shift_indicationY = currY
                            mated_shift_indicationZ = currZ
                            newz = currZ + correction_shiftz
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                            correction_shiftz = newz - currZ
                            prev_rotation_axis = "X"

                    #Mating Gear
                    if preX == currX and preY != currY and preZ == currZ:

                        if currY > preY:
                            newy = preY + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                            correction_shifty = newy - currY
                            prev_rotation_axis = "X"
                        if currY < preY:
                            newy = preX - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                            correction_shifty = newy - currY
                            prev_rotation_axis = "X"

                    if preX == currX and preY == currY and preZ != currZ:

                        if currZ > preZ:
                            newz = preZ + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                            correction_shiftz = newz - currZ
                            prev_rotation_axis = "X"
                        if currZ < preZ:
                            newz = preX - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                            correction_shiftz = newz - currZ
                            prev_rotation_axis = "X"


                    elif preX == currX and preY != currY and preZ != currZ:
                        if currZ > preZ and currY < preY:
                            if abs(mated_shift_indicationY - preY) == 0:
                                newy = preY - Rpre - Rcurr
                            if abs(mated_shift_indicationY - preY) != 0:
                                newy = currY + correction_shifty  # 4cases because the imposed shift or the resultant correction shift could be in any of the values of z,y
                            if abs(mated_shift_indicationZ - preZ) == 0:
                                newz = preZ + Rpre + Rcurr
                            if abs(mated_shift_indicationZ - preZ) != 0:
                                newz = currZ + correction_shiftz

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, newz, Rcurr
                            correction_shiftz = newz - currZ
                            correction_shifty = newy - currY
                            prev_rotation_axis = "X"

                        if currZ > preZ and currY > preY:
                            if abs(mated_shift_indicationY - preY) == 0:
                                newy = preY + Rpre + Rcurr
                            if abs(mated_shift_indicationY - preY) != 0:
                                newy = currY + correction_shifty  # 4cases because the imposed shift or the resultant correction shift could be in any of the values of z,y
                            if abs(mated_shift_indicationZ - preZ) == 0:
                                newz = preZ + Rpre + Rcurr
                            if abs(mated_shift_indicationZ - preZ) != 0:
                                newz = currZ + correction_shiftz

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, newz, Rcurr
                            correction_shiftz = newz - currZ
                            correction_shifty = newy - currY
                            prev_rotation_axis = "X"

                        if currZ < preZ and currY > preY:
                            if abs(mated_shift_indicationY - preY) == 0:
                                newy = preY + Rpre + Rcurr
                            if abs(mated_shift_indicationY - preY) != 0:
                                newy = currY + correction_shifty  # 4cases because the imposed shift or the resultant correction shift could be in any of the values of z,y
                            if abs(mated_shift_indicationZ - preZ) == 0:
                                newz = preZ - Rpre - Rcurr
                            if abs(mated_shift_indicationZ - preZ) != 0:
                                newz = currZ + correction_shiftz

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, newz, Rcurr
                            correction_shiftz = newz - currZ
                            correction_shifty = newy - currY
                            prev_rotation_axis = "X"

                        if currZ < preZ and currY < preY:
                            if abs(mated_shift_indicationY - preY) == 0:
                                newy = preY - Rpre - Rcurr
                            if abs(mated_shift_indicationY - preY) != 0:
                                newy = currY + correction_shifty  # 4cases because the imposed shift or the resultant correction shift could be in any of the values of z,y
                            if abs(mated_shift_indicationZ - preZ) == 0:
                                newz = preZ - Rpre - Rcurr
                            if abs(mated_shift_indicationZ - preZ) != 0:
                                newz = currZ + correction_shiftz

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, newz, Rcurr
                            correction_shiftz = newz - currZ
                            correction_shifty = newy - currY
                            prev_rotation_axis = "X"


            precedent_gears_info[gear_id] = {
                'X': currX,
                'Y': currY,
                'Z': currZ,
                'R': Rcurr,
                'FinalMX': final_move_X,
                'FinalMY': final_move_Y,
                'FinalMZ': final_move_Z,
            }


        modelica_code_connection += f"end {model_title};\n"
        part_construction.append(modelica_code_part)
        connection_construction.append(modelica_code_connection)
        parts_lines = "\n".join(part_construction)
        connections_lines = "\n".join(connection_construction)

        with open('output.txt', 'w') as file:
            file.write(parts_lines)

        with open('output.txt', 'a') as file:
            file.write(connections_lines)

        with open('output.txt', 'r') as file:

            result = print(file.read())

        return result


# Example usage:
modelica_code = parse_ldraw_into_modelica(ldraw_file, model_title)

print(modelica_code)
