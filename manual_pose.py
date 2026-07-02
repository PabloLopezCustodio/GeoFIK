import re
import tkinter as tk
from tkinter import ttk

import mujoco
import mujoco.viewer
import numpy as np

# Matches lines like "q_1 = -1.20522".
PASTE_PATTERN = re.compile(r"q_(\d)\s*=\s*(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)")


def main() -> None:
    assert mujoco.__version__ >= "3.1.0", "Please upgrade to mujoco 3.1.0 or later."

    model = mujoco.MjModel.from_xml_path("mjctrl/franka_emika_panda/scene.xml")
    data = mujoco.MjData(model)

    joint_names = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint7",
    ]
    dof_ids = np.array([model.joint(name).id for name in joint_names])
    joint_ranges = model.jnt_range[dof_ids]

    key_name = "home"
    key_id = model.key(key_name).id
    q0 = model.key(key_name).qpos[dof_ids].copy()

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        show_left_ui=False,
        show_right_ui=True,
    ) as viewer:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
        viewer.sync()

        root = tk.Tk()
        root.title("Panda Manual Joint Angles (rad)")

        values = list(q0)
        entries = []
        sliders = []
        updating = False

        def refresh_widgets() -> None:
            nonlocal updating
            updating = True
            for entry, slider, value in zip(entries, sliders, values):
                entry.delete(0, tk.END)
                entry.insert(0, f"{value:.4f}")
                slider.set(value)
            updating = False

        def apply_current() -> None:
            q = np.clip(values, joint_ranges[:, 0], joint_ranges[:, 1])
            clipped = not np.allclose(q, values)
            values[:] = q.tolist()

            data.qpos[dof_ids] = q
            mujoco.mj_forward(model, data)
            viewer.sync()

            refresh_widgets()
            status_var.set("Clipped to joint limits" if clipped else "Applied")

        def apply_from_entries(*_args) -> None:
            if updating:
                return
            for i, entry in enumerate(entries):
                try:
                    values[i] = float(entry.get())
                except ValueError:
                    status_var.set(f"Invalid value for {joint_names[i]}")
                    return
            apply_current()

        def apply_from_slider(i, value) -> None:
            if updating:
                return
            values[i] = float(value)
            apply_current()

        def reset_to_home() -> None:
            values[:] = q0.tolist()
            apply_current()

        def apply_from_paste(*_args) -> None:
            if updating:
                return
            text = paste_text.get("1.0", tk.END)
            found = {int(idx): float(val) for idx, val in PASTE_PATTERN.findall(text)}
            if all(i in found for i in range(1, len(joint_names) + 1)):
                for i in range(len(joint_names)):
                    values[i] = found[i + 1]
                apply_current()
                status_var.set("Applied pasted values")
            elif found:
                status_var.set("Pasted text is missing some joints (need q_1..q_7)")

        main_frame = ttk.Frame(root, padding=10)
        main_frame.grid(row=0, column=0, sticky="nsew")
        main_frame.columnconfigure(2, weight=1)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        status_var = tk.StringVar(value="Ready")

        # Building the sliders below triggers their `command` callback as soon as
        # `.set()` is called, before all widgets exist, so suppress it during setup.
        updating = True
        for i, name in enumerate(joint_names):
            lo, hi = joint_ranges[i]
            ttk.Label(main_frame, text=f"{name}  [{lo:.3f}, {hi:.3f}]").grid(
                row=i, column=0, sticky="w", padx=(0, 8), pady=2
            )
            entry = ttk.Entry(main_frame, width=10)
            entry.insert(0, f"{q0[i]:.4f}")
            entry.grid(row=i, column=1, padx=(0, 8), pady=2)
            entry.bind("<Return>", apply_from_entries)
            entries.append(entry)

            slider = ttk.Scale(
                main_frame,
                from_=lo,
                to=hi,
                orient=tk.HORIZONTAL,
                command=lambda v, i=i: apply_from_slider(i, v),
            )
            slider.set(q0[i])
            slider.grid(row=i, column=2, sticky="ew", pady=2)
            sliders.append(slider)
        updating = False

        button_row = len(joint_names)
        ttk.Button(main_frame, text="Apply", command=apply_from_entries).grid(
            row=button_row, column=0, pady=(10, 0)
        )
        ttk.Button(main_frame, text="Reset to home", command=reset_to_home).grid(
            row=button_row, column=1, columnspan=2, sticky="w", pady=(10, 0)
        )

        ttk.Label(main_frame, textvariable=status_var).grid(
            row=button_row + 1, column=0, columnspan=3, sticky="w", pady=(6, 0)
        )

        ttk.Separator(main_frame, orient=tk.HORIZONTAL).grid(
            row=button_row + 2, column=0, columnspan=3, sticky="ew", pady=(10, 6)
        )
        ttk.Label(main_frame, text="Paste joint angles (q_1 = ... .. q_7 = ...):").grid(
            row=button_row + 3, column=0, columnspan=3, sticky="w"
        )
        paste_text = tk.Text(main_frame, height=8, width=40)
        paste_text.grid(
            row=button_row + 4, column=0, columnspan=3, sticky="nsew", pady=(4, 0)
        )
        main_frame.rowconfigure(button_row + 4, weight=1)

        def on_paste_modified(_event) -> None:
            if paste_text.edit_modified():
                apply_from_paste()
                paste_text.edit_modified(False)

        paste_text.bind("<<Modified>>", on_paste_modified)

        def poll_viewer() -> None:
            if not viewer.is_running():
                root.destroy()
                return
            root.after(100, poll_viewer)

        root.after(100, poll_viewer)
        root.mainloop()


if __name__ == "__main__":
    main()
