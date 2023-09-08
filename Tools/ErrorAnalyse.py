"""
"""
import yaml
import sys
from ErrorAnalyser import ErrorAnalyser
import prettytable as pt
import numpy as np


def load_config(yml_path: str):
    yml = yaml.full_load(open(yml_path, 'r'))
    result_path = yml["Output-Config"]["file-path"]
    truth = yml["reference"]
    return result_path, truth


usage = """
python ErrorAnalyse.py {config}.yml {2d error} {3d error}
usage :python ErrorAnalyse.py config.yml 0.04 0.05
"""
if __name__ == '__main__':
    if len(sys.argv) < 3: print(usage)
    print("Error Analyse program start")
    yml_path = sys.argv[1]
    target_path, truth_path = load_config(yml_path)
    print("ground truth:{}".format(truth_path))
    print("target file:{}".format(target_path))
    column_index = [0, 1, 2, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
    e = ErrorAnalyser(target_path, truth_path, column_index, write2file=False)
    e.analyse(_3d=True, _2d=True)

    outage_index = None
    table = pt.PrettyTable()
    table.field_names = [" ", "x error /m", "y error/m", "z error/m"]
    table.border = True
    table.junction_char = "|"
    table.horizontal_char = '-'
    table.vertical_char = '|'
    fmt = "%.4f"
    header2 = ["position error(m)", "velocity error(m/s)", "attitude error(deg)"]
    for k in range(3):
        indexs = column_index[3 * k + 2:3 * k + 5]
        if indexs[0] > 0 and indexs[2] > 0 and indexs[1] > 0:
            table.add_row([header2[k] + " 1-$\\sigma$", fmt % e.errs[indexs[0]].first_sigma,
                           fmt % e.errs[indexs[1]].first_sigma, fmt % e.errs[indexs[2]].first_sigma])
            table.add_row(
                [header2[k] + " 2-$\\sigma$", fmt % e.errs[indexs[0]].second_sigma,
                 fmt % e.errs[indexs[1]].second_sigma,
                 fmt % e.errs[indexs[2]].second_sigma])

            table.add_row([header2[k] + " rms", fmt % e.errs[indexs[0]].rms, fmt % e.errs[indexs[1]].rms,
                           fmt % e.errs[indexs[2]].rms])
    table.add_row(["2D error" + " 1-$\\sigma$", fmt % e.error_2d.first_sigma, "", ""])
    table.add_row(["2D error" + " 2-$\\sigma$", fmt % e.error_2d.second_sigma, "", ""])
    table.add_row(["2D error" + " RMS$", fmt % e.error_2d.rms, "", ""])
    table.add_row(["3D error" + " 1-$\\sigma$", fmt % e.error_3d.first_sigma, "", ""])
    table.add_row(["3D error" + " 2-$\\sigma$", fmt % e.error_3d.second_sigma, "", ""])
    table.add_row(["3D error" + " RMS $\\sigma$", fmt % e.error_3d.rms, "", ""])
    if column_index[11] > 0 and outage_index is not None:
        table.add_row(["Outage 2D", fmt % (np.mean(e.error_2d.error[outage_index])), "", ""])
        table.add_row(["Outage 3D", fmt % (np.mean(e.error_3d.error[outage_index])), "", ""])
        table.add_row(
            ["Outage Pos", fmt % (np.sqrt(np.mean(e.errs[column_index[2]].error[outage_index] ** 2))),
             fmt % (np.sqrt(np.mean(e.errs[column_index[3]].error[outage_index] ** 2))),
             fmt % (np.sqrt(np.mean(e.errs[column_index[4]].error[outage_index] ** 2))),
             ])
        if column_index[8] > 0 and column_index[9] > 0 and column_index[10] > 0:
            table.add_row(
                ["Outage Atti", fmt % (np.sqrt(np.mean(e.errs[column_index[8]].error[outage_index] ** 2))),
                 fmt % (np.sqrt(np.mean(e.errs[column_index[9]].error[outage_index] ** 2))),
                 fmt % (np.sqrt(np.mean(e.errs[column_index[10]].error[outage_index] ** 2))),
                 ])
    print(table)
    if e.error_3d.second_sigma > float(sys.argv[3]):
        print("3d 2-sigma error: {:.4} > {}".format(e.error_3d.second_sigma, sys.argv[3]))
        exit(1)
    if e.error_2d.second_sigma > float(sys.argv[2]):
        print("2d 2-sigma error:  {:.4} > {}".format(e.error_3d.second_sigma, sys.argv[2]))
        exit(1)
    print("finished")
    exit(0)
