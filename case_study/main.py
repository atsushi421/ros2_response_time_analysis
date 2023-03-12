from plot_analysis import generate_charts_for_paper
from move_base import models
from pycpa.graph import graph_system

if __name__ == "__main__":
    for num_res in [1, 2]:
        for mname, m in models.items():
            s = m(num_res)
            graph_system(s, filename='{}-{}.pdf'.format(mname, num_res),
                         dotout='{}-{}.dot'.format(mname, num_res), show=False)

    generate_charts_for_paper()
