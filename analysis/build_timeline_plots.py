import os
import json
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


for mode in ["single-mission", "multi-mission/atstart-income", "multi-mission/online-income"]:

    with open(f"../output/{mode}/robot_data.json", 'r') as f:
        robot_data = json.load(f)

    def plot_robot_gantt(mission_data, fname=None):

        def normalize_task_name(task):
            task = task.replace("_", " ")
            task = task.replace("^", " ")
            return task.title()

        robots_data = mission_data["robots"]
        mission_duration = mission_data["mission_duration"]

        # Ordine robot: robot_1, robot_2, ...
        def robot_key(name):
            try:
                return int(name.split("_")[1])
            except:
                return name

        robot_names = sorted(robots_data.keys(), key=robot_key)

        # Lista dei task distinti
        all_tasks = []
        for robot in robot_names:
            for start, duration, task in robots_data[robot]["non_idle_intervals"]:
                if task != "":
                    all_tasks.append(normalize_task_name(task))

        unique_tasks = sorted(set(all_tasks))

        # Colormap standard di matplotlib
        cmap = plt.get_cmap("tab20")
        task_to_color = {
            task: cmap(i % 20) for i, task in enumerate(unique_tasks)
        }

        fig_height = max(2, 0.75* len(robot_names))
        fig, ax = plt.subplots(figsize=(14, fig_height))

        bar_height = 0.8
        y_positions = {}

        for i, robot in enumerate(robot_names):
            y = i
            y_positions[robot] = y

            intervals = [interval for interval in robots_data[robot]["non_idle_intervals"] if interval[2] != ""]

            for start, duration, task in intervals:
                color = task_to_color.get(normalize_task_name(task), "tab:blue")

                ax.broken_barh(
                    [(start, duration)],
                    (y - bar_height / 2, bar_height),
                    facecolors=color,
                    edgecolors="black",
                    linewidth=0.8
                )

                # Etichetta task dentro la barra, solo se abbastanza lunga
                if duration > 3:
                    ax.text(
                        start + duration / 2,
                        y,
                        normalize_task_name(task),
                        ha="center",
                        va="center",
                        fontsize=8,
                        color="black",
                        clip_on=True
                    )

        ax.set_xlim(0, mission_duration)
        ax.set_ylim(-1, len(robot_names))
        ax.set_xlabel("Time from mission start (s)")
        ax.set_ylabel("Robots")
        ax.set_yticks(range(len(robot_names)))
        ax.set_yticklabels([robot_name.replace("robot_", "Robot ") for robot_name in robot_names])

        ax.grid(True, axis="x", linestyle="--", alpha=0.5)
        ax.set_axisbelow(True)

        # Legenda task -> colore
        legend_patches = [
            mpatches.Patch(color=task_to_color[task], label=task)
            for task in unique_tasks
        ]

        if legend_patches:
            ax.legend(
                handles=legend_patches,
                title="Tasks",
                bbox_to_anchor=(1.02, 1),
                loc="upper left",
                borderaxespad=0.
            )

        if fname:
            plt.savefig(fname, bbox_inches="tight", format="pdf")
        plt.tight_layout()
        # plt.show()
        plt.close()

    # test_id = "test_10_10_t_cleaning_i0_multi3"
    # plot_non_idle_timeline(data[test_id])


    if not os.path.exists(f"../output/{mode}/images/timelines/"):
        os.makedirs(f"../output/{mode}/images/timelines/")

    for test_id in robot_data.keys():
        plot_robot_gantt(robot_data[test_id], fname=f"../output/{mode}/images/timelines/{test_id}.pdf")
