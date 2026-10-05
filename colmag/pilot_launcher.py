"""Small launcher form for the observing-only virtual task pilot."""

import tkinter as tk
from tkinter import messagebox, ttk


class PilotPanel(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent
        self.title('MagPilot · virtual task')
        self.configure(bg='#f5f7fa')
        self.resizable(False, False)
        self.participant = tk.StringVar(value='pilot')
        self.condition = tk.StringVar(value='practice')
        self.magnet_count = tk.StringVar(value='none')
        self.tolerance = tk.StringVar(value='20')
        self.dwell = tk.StringVar(value='0.5')
        self.repetitions = tk.StringVar(value='1')
        self.notes = tk.StringVar(value='')
        frame = tk.Frame(self, bg='#f5f7fa', padx=24, pady=22)
        frame.pack()
        tk.Label(frame, text='Virtual target-reaching pilot', bg='#f5f7fa',
                 fg='#1d1d1f', font=parent.f_h).grid(row=0, column=0, columnspan=2, sticky='w')
        tk.Label(frame, text='Simulation → Robot → Arm nodes → Interface first.\n'
                            'The pilot records actual flange feedback; controls stay\n'
                            'in the Interface window. Condition is a log label.',
                 bg='#f5f7fa', fg='#677482', justify='left',
                 font=parent.f_small).grid(row=1, column=0, columnspan=2,
                                            sticky='w', pady=(8, 16))
        fields = [('Participant ID', self.participant), ('Condition', self.condition),
                  ('Magnet stack', self.magnet_count), ('Tolerance (mm)', self.tolerance),
                  ('Hold (s)', self.dwell), ('Repetitions / target', self.repetitions),
                  ('Notes', self.notes)]
        for row, (label, value) in enumerate(fields, start=2):
            tk.Label(frame, text=label, bg='#f5f7fa', fg='#1d1d1f',
                     font=parent.f_body).grid(row=row, column=0, sticky='w', padx=(0, 14), pady=5)
            if label == 'Magnet stack':
                widget = ttk.Combobox(frame, textvariable=value, values=('none', '1', '2', '3'),
                                      state='readonly', width=21)
            else:
                widget = ttk.Entry(frame, textvariable=value, width=24)
            widget.grid(row=row, column=1, sticky='ew', pady=5)
        ttk.Button(frame, text='Open pilot', command=self.start).grid(
            row=9, column=1, sticky='e', pady=(16, 0))

    def start(self):
        try:
            values = dict(participant_id=self.participant.get().strip(),
                          condition=self.condition.get().strip(),
                          magnet_count=(None if self.magnet_count.get() == 'none'
                                        else int(self.magnet_count.get())),
                          tolerance_mm=float(self.tolerance.get()),
                          dwell_s=float(self.dwell.get()),
                          repetitions=int(self.repetitions.get()), notes=self.notes.get())
            if not values['participant_id'] or not values['condition']:
                raise ValueError('Enter a participant ID and a condition label.')
        except ValueError as exc:
            messagebox.showerror('Virtual task', str(exc), parent=self)
            return
        self.parent.start_virtual_task(**values)
