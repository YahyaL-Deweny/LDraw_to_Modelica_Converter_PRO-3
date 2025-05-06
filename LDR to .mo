ldraw_file = 'Gear_wheel.ldr'  # LeoCAD file name (changeable)

model_title = ldraw_file[:-4]


def parse_ldraw_into_modelica(ldraw_file, model_title):

    part_construction = []
    connection_construction = []

    # Parsing process starts to extract and save position and rotation info
    with (open(ldraw_file, 'r') as file):
        modelica_code_part = f"model {model_title}\n"
        modelica_code_connection = "equation\n"

        gear_train = []

        preX, preY, preZ, Rpre = 0, 0, 0, 0

        for line in file:

            line = line.strip().split()

            if line[0] == '1' and line[1] == '7':
                part_id = line[14]
                position = list(map(float, line[2:5]))
                rotation = list(map(float, line[5:14]))

                # append info to the list of all the gears
                gear_train.append({
                    'part_id': part_id,
                    'position': position,
                    'rotation': rotation,
                })

        part_id_to_idx = {}
        for idx, part in enumerate(gear_train):
            part_id_int = part['part_id'].replace('.dat', '')
            part_id_to_idx[part_id_int] = idx



        for idx, part in enumerate(gear_train):               # So that we can access the value of the index itself and the value in that index
            currX = part['position'][0]
            # real_currX = x_position0.0004
            currZ = part['position'][1]
            # real_currZ = z_position*0.0004
            currY = part['position'][2]
            # real_currY = y_position*0.0004

            first_rotation = part['rotation'][0]
            second_rotation = part['rotation'][1]
            third_rotation = part['rotation'][2]
            fourth_rotation = part['rotation'][3]
            fifth_rotation = part['rotation'][4]
            sixth_rotation = part['rotation'][5]
            seventh_rotation = part['rotation'][6]
            eighth_rotation = part['rotation'][7]
            ninth_rotation = part['rotation'][8]



            gear_char = {  # this dictionary should include parts' specifications (adjustable)
                '6542a.dat': {"id": part_id_to_idx['6542a'], "Diameter": 0.0175, "depth": 0.008, "ratio": "N/A"},
                '2740.dat': {"id": part_id_to_idx['2740'], "Diameter": 0.0259, "depth": 0.0077, "ratio": 1.48},
                '65413.dat': {"id": part_id_to_idx['65413'], "Diameter": 0.03, "depth": 0.008, "ratio": 1.1583011583011583011583011583012},
                '34432.dat': {"id": part_id_to_idx['34432'], "Diameter": 0.0482, "depth": 0.008, "ratio": 1.6066666666666666666666666666667},
                '60.dat': {"id": part_id_to_idx['60'], "Diameter": 0.0254, "depth": 0.008, "ratio": 0.52697095435684647302904564315353}
            }

            global gear_name
            gear_name = part['part_id']
            gear_ratio = gear_char[gear_name]["ratio"]
            gear_dia = gear_char[gear_name]["Diameter"] * (1000)
            Rcurr = gear_dia * (0.5)
            gear_thick = gear_char[gear_name]["depth"]
            gear_id = gear_char[gear_name][ "id"]  # the id of the part "gear" in the pre-constructed dictionary based on part name in LeoCAD
            balance = gear_id - 1

            body_name = f"cyl{gear_id}"
            body_position_name = f"fixedTranslation{gear_id}"
            actuate_revolute_name = f"actuatedRevolute_{gear_id}"
            default_rotation_axis_y_tox = '1'

            last_idx = len(gear_train) - 1


            if gear_id == 0:

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
                if first_rotation == 1 and second_rotation == 0 and third_rotation == 0 and fourth_rotation == 0 and fifth_rotation == 1 and sixth_rotation == 0 and seventh_rotation == 0 and eighth_rotation == 0 and ninth_rotation == 1:
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{{default_rotation_axis_y_tox}, 0, 0}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{{gear_thick}, 0, 0}});\n"

                #Case#2 rotation around z(y in modelica)
                if (first_rotation != 1 or second_rotation != 0 or third_rotation != 0) and (fourth_rotation == 0 and fifth_rotation == 1 and sixth_rotation == 0 and seventh_rotation == 0 and eighth_rotation == 0 and ninth_rotation == 1):
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{0, 0, {default_rotation_axis_y_tox}}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{0, {gear_thick}, 0}});\n"

                #Case#3 rotation around x(z in modelica)
                elif fourth_rotation != 0 or fifth_rotation != 1 or sixth_rotation != 0:
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{0,{default_rotation_axis_y_tox}, 0}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{0,0, {gear_thick}}});\n"

            if gear_id != last_idx and gear_id != 0:


                modelica_code_part += f"  Modelica.Mechanics.Rotational.Components.IdealGear idealGear{balance}(ratio = {gear_ratio});\n"
                modelica_code_connection += f"  connect({body_position_name}.frame_a, world.frame_b);\n"
                modelica_code_connection += f"  connect({body_position_name}.frame_b, {actuate_revolute_name}.frame_a);\n"
                modelica_code_connection += f"  connect({actuate_revolute_name}.frame_b, {body_name}.frame_a);\n"
                modelica_code_connection += f"  connect(idealGear{balance}.flange_a, actuatedRevolute_{balance}.axis);\n"
                modelica_code_connection += f"  connect(idealGear{balance}.flange_b, {actuate_revolute_name}.axis);\n"

                #Case#1 default_orientation (the gear rotates around Y-Axis in LeoCAD & X-Axis in OM)
                if first_rotation == 1 and second_rotation == 0 and third_rotation == 0 and fourth_rotation == 0 and fifth_rotation == 1 and sixth_rotation == 0 and seventh_rotation == 0 and eighth_rotation == 0 and ninth_rotation == 1:
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{{default_rotation_axis_y_tox}, 0, 0}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{{gear_thick}, 0, 0}});\n"

                    if preX != currX and preY == currY and preZ == currZ:
                        if preX > 0 and currX > 0:
                            if currX > preX:
                                newx = preX + Rpre + Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                            if currX < preX:
                                newx = preX - Rpre - Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                        if preX < 0 and currX < 0:
                            if abs(currX) > abs(preX):
                                newx = preX - Rpre - Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                            if abs(currX) < abs(preX):
                                newx = preX + Rpre + Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                        if preX > 0 and currX < 0:
                            newx = preX - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                        if currX > 0 and preX < 0:
                            newx = preX + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                    elif preX == currX and preY == currY and preZ != currZ:
                        if preZ > 0 and currZ > 0:
                            if currZ > preZ:
                                newz = preZ + Rpre + Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                            if currX < preZ:
                                newz = preZ - Rpre - Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                        if preZ < 0 and currZ < 0:
                            if abs(currZ) > abs(preZ):
                                newz = preZ - Rpre - Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                            if abs(currZ) < abs(preZ):
                                newz = preZ + Rpre + Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                        if preZ > 0 and currZ < 0:
                            newz = preZ - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                        if currZ > 0 and preZ < 0:
                            newz = preZ + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr


                    else:
                        newx = preX
                        newz = preZ
                        modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                        preX, preY, preZ, Rpre = newx, currY, newz ,Rcurr
                #Case#2 rotation around z(y in modelica)
                if first_rotation != 1 or second_rotation != 0 or third_rotation != 0:
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{0, 0, {default_rotation_axis_y_tox}}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{0, 0, {gear_thick}}});\n"

                    if preX == currX and preY != currY and preZ == currZ:
                        if preY > 0 and currY > 0:
                            if currY > preY:
                                newy = preY + Rpre + Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                            if currY < preY:
                                newy = preY - Rpre - Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                        if preY < 0 and currY < 0:
                            if abs(currY) > abs(preY):
                                newy = preY - Rpre - Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                            if abs(currY) < abs(preY):
                                newy = preY + Rpre + Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                        if preY > 0 and currY < 0:
                            newy = preY - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                        if currY > 0 and preY < 0:
                            newy = preY + Rpre + Rcurr
                            newz = preY + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr

                    elif preX == currX and preY == currY and preZ != currZ:
                        if preZ > 0 and currZ > 0:
                            if currZ > preZ:
                                newz = preZ + Rpre + Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                            if currX < preZ:
                                newz = preZ - Rpre - Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                        if preZ < 0 and currZ < 0:
                            if abs(currZ) > abs(preZ):
                                newz = preZ - Rpre - Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                            if abs(currZ) < abs(preZ):
                                newz = preZ + Rpre + Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                        if preZ > 0 and currZ < 0:
                            newz = preZ - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                        if currZ > 0 and preZ < 0:
                            newz = preZ + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                    else:
                        newy = preY - Rpre - Rcurr
                        newz = preZ - Rpre - Rcurr
                        modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {newz}, {currX}}});\n"
                        preX, preY, preZ, Rpre = currX, newy, newz, Rcurr

                #Case#3 rotation around x(z in modelica)
                elif fourth_rotation != 0 or fifth_rotation != 1 or sixth_rotation != 0:
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{0,{default_rotation_axis_y_tox}, 0}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{0, {gear_thick}, 0}});\n"

                    if preX != currX and preY == currY and preZ == currZ:
                        if preX > 0 and currX > 0:
                            if currX > preX:
                                newx = preX + Rpre + Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                            if currX < preX:
                                newx = preX - Rpre - Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                        if preX < 0 and currX < 0:
                            if abs(currX) > abs(preX):
                                newx = preX - Rpre - Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                            if abs(currX) < abs(preX):
                                newx = preX + Rpre + Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                        if preX > 0 and currX < 0:
                            newx = preX - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                        if currX > 0 and preX < 0:
                            newx = preX + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                    if preX == currX and preY != currY and preZ == currZ:

                        if preY > 0 and currY > 0:

                            if currY > preY:
                                newy = preY + Rpre + Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr

                            if currY < preY:
                                newy = preY - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr

                        if preY < 0 and currY < 0:

                            if abs(currY) > abs(preY):
                                newy = preY - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr

                            if abs(currY) < abs(preY):
                                newy = preY + Rpre + Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr

                        if preY > 0 and currY < 0:
                            newy = preY - Rpre - Rcurr

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"

                            preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr

                        if currY > 0 and preY < 0:
                            newy = preY + Rpre + Rcurr

                            newz = preZ + Rpre + Rcurr

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {newz}, {currX}}});\n"

                            preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr


                    elif preX == currX and preY == currY and preZ != currZ:

                        if preZ > 0 and currZ > 0:

                            if currZ > preZ:
                                newz = preZ + Rpre + Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                            if currX < preZ:
                                newz = preZ - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                        if preZ < 0 and currZ < 0:

                            if abs(currZ) > abs(preZ):
                                newz = preZ - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                            if abs(currZ) < abs(preZ):
                                newz = preZ + Rpre + Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                        if preZ > 0 and currZ < 0:
                            newz = preZ - Rpre - Rcurr

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                        if currZ > 0 and preZ < 0:
                            newz = preZ + Rpre + Rcurr

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr


                    else:

                        newy = preY - Rpre - Rcurr

                        newx = preX - Rpre - Rcurr

                        modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {newx}}});\n"

                        preX, preY, preZ, Rpre = newx, newy, currZ, Rcurr


            if gear_id == last_idx:
                
                modelica_code_part += f"  Modelica.Mechanics.Rotational.Components.IdealGear idealGear{balance}(ratio = {gear_ratio});\n"

                modelica_code_connection += f"  connect({body_position_name}.frame_a, world.frame_b);\n"
                modelica_code_connection += f"  connect({body_position_name}.frame_b, {actuate_revolute_name}.frame_a);\n"
                modelica_code_connection += f"  connect({actuate_revolute_name}.frame_b, {body_name}.frame_a);\n"
                modelica_code_connection += f"  connect(idealGear{balance}.flange_a, actuatedRevolute_{balance}.axis);\n"
                modelica_code_connection += f"  connect(idealGear{balance}.flange_b, {actuate_revolute_name}.axis);\n"

                #Case#1 default_orientation (the gear rotates around Y-Axis in LeoCAD & X-Axis in OM)
                if first_rotation == 1 and second_rotation == 0 and third_rotation == 0 and fourth_rotation == 0 and fifth_rotation == 1 and sixth_rotation == 0 and seventh_rotation == 0 and eighth_rotation == 0 and ninth_rotation == 1:
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{{default_rotation_axis_y_tox}, 0, 0}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{{gear_thick}, 0, 0}});\n"

                if preX != currX and preY == currY and preZ == currZ:
                    if preX > 0 and currX > 0:
                        if currX > preX:
                            newx = preX + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                        if currX < preX:
                            newx = preX - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                    if preX < 0 and currX < 0:
                        if abs(currX) > abs(preX):
                            newx = preX - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                        if abs(currX) < abs(preX):
                            newx = preX + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                    if preX > 0 and currX < 0:
                        newx = preX - Rpre - Rcurr
                        modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                        preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                    if currX > 0 and preX < 0:
                        newx = preX + Rpre + Rcurr
                        modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                        preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                elif preX == currX and preY == currY and preZ != currZ:
                    if preZ > 0 and currZ > 0:
                        if currZ > preZ:
                            newz = preZ + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                        if currX < preZ:
                            newz = preZ - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                    if preZ < 0 and currZ < 0:
                        if abs(currZ) > abs(preZ):
                            newz = preZ - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                        if abs(currZ) < abs(preZ):
                            newz = preZ + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                    if preZ > 0 and currZ < 0:
                        newz = preZ - Rpre - Rcurr
                        modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                        preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                    if currZ > 0 and preZ < 0:
                        newz = preZ + Rpre + Rcurr
                        modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                        preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                else:
                    newx = preX - Rpre - Rcurr
                    newz = preZ - Rpre - Rcurr
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {newx}}});\n"
                    preX, preY, preZ, Rpre = newx, currY, newz, Rcurr

                 #Case#2 rotation around z(y in modelica)
                if first_rotation != 1 or second_rotation != 0 or third_rotation != 0:
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{0, 0, {default_rotation_axis_y_tox}}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{0, 0, {gear_thick}}});\n"

                    if preX == currX and preY != currY and preZ == currZ:
                        if preY > 0 and currY > 0:
                            if currY > preY:
                                newy = preY + Rpre + Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                            if currY < preY:
                                newy = preY - Rpre - Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                        if preY < 0 and currY < 0:
                            if abs(currY) > abs(preY):
                                newy = preY - Rpre - Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                            if abs(currY) < abs(preY):
                                newy = preY + Rpre + Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                        if preY > 0 and currY < 0:
                            newy = preY - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr
                        if currY > 0 and preY < 0:
                            newy = preY + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr

                    elif preX == currX and preY == currY and preZ != currZ:
                        if preZ > 0 and currZ > 0:
                            if currZ > preZ:
                                newz = preZ + Rpre + Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                            if currX < preZ:
                                newz = preZ - Rpre - Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                        if preZ < 0 and currZ < 0:
                            if abs(currZ) > abs(preZ):
                                newz = preZ - Rpre - Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                            if abs(currZ) < abs(preZ):
                                newz = preZ + Rpre + Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                        if preZ > 0 and currZ < 0:
                            newz = preZ - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr
                        if currZ > 0 and preZ < 0:
                            newz = preZ + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"
                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                    else:
                        newy = preY - Rpre - Rcurr
                        newz = preZ - Rpre - Rcur
                        modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {newz}, {currX}}});\n"
                        preX, preY, preZ, Rpre = currX, newy, newz, Rcurr

                # Case#3 rotation around x(z in modelica)
                elif fourth_rotation != 0 or fifth_rotation != 1 or sixth_rotation != 0:
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Joints.Revolute {actuate_revolute_name}(n = {{0,{default_rotation_axis_y_tox}, 0}}, useAxisFlange = true);\n"
                    modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.BodyCylinder {body_name}(color = {{0, 128, 0}}, diameter = {gear_dia}, r = {{0, {gear_thick}, 0}});\n"

                    if preX != currX and preY == currY and preZ == currZ:
                        if preX > 0 and currX > 0:
                            if currX > preX:
                                newx = preX + Rpre + Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                            if currX < preX:
                                newx = preX - Rpre - Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                        if preX < 0 and currX < 0:
                            if abs(currX) > abs(preX):
                                newx = preX - Rpre - Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                            if abs(currX) < abs(preX):
                                newx = preX + Rpre + Rcurr
                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                                preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                        if preX > 0 and currX < 0:
                            newx = preX - Rpre - Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr
                        if currX > 0 and preX < 0:
                            newx = preX + Rpre + Rcurr
                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {currZ}, {newx}}});\n"
                            preX, preY, preZ, Rpre = newx, currY, currZ, Rcurr

                    if preX == currX and preY != currY and preZ == currZ:

                        if preY > 0 and currY > 0:

                            if currY > preY:
                                newy = preY + Rpre + Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr

                            if currY < preY:
                                newy = preY - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr

                        if preY < 0 and currY < 0:

                            if abs(currY) > abs(preY):
                                newy = preY - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr

                            if abs(currY) < abs(preY):
                                newy = preY + Rpre + Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr

                        if preY > 0 and currY < 0:
                            newy = preY - Rpre - Rcurr

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {currX}}});\n"

                            preX, preY, preZ, Rpre = currX, newy, currZ, Rcurr

                        if currY > 0 and preY < 0:
                            newy = preY + Rpre + Rcurr

                            newz = preZ + Rpre + Rcurr

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {newz}, {currX}}});\n"

                            preX, preY, preZ, Rpre = currX, newy, newz, Rcurr


                    elif preX == currX and preY == currY and preZ != currZ:

                        if preZ > 0 and currZ > 0:

                            if currZ > preZ:
                                newz = preZ + Rpre + Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                            if currZ < preZ:
                                newz = preZ - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                        if preZ < 0 and currZ < 0:

                            if abs(currZ) > abs(preZ):
                                newz = preZ - Rpre - Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                            if abs(currZ) < abs(preZ):
                                newz = preZ + Rpre + Rcurr

                                modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                                preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                        if preZ > 0 and currZ < 0:
                            newz = preZ - Rpre - Rcurr

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr

                        if currZ > 0 and preZ < 0:
                            newz = preZ + Rpre + Rcurr

                            modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{currY}, {newz}, {currX}}});\n"

                            preX, preY, preZ, Rpre = currX, currY, newz, Rcurr


                    else:

                        newy = preY - Rpre - Rcurr

                        newx = preX - Rpre - Rcur

                        modelica_code_part += f"  Modelica.Mechanics.MultiBody.Parts.FixedTranslation {body_position_name}(animation = false, r = {{{newy}, {currZ}, {newx}}});\n"

                        preX, preY, preZ, Rpre = newx, newy, currZ, Rcurr


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
