# Logic for handling PID parameter updates and tuning.

from server.database.dynamodb import update_pid_values


def handle_inner_pid_update(device_id, pg, dg, ig, sp):
    update_pid_values(
        device_id=device_id,
        loop="inner",
        pg=pg,
        dg=dg,
        ig=ig,
        sp=sp,
        rot=0
    )
    print(f"[PID] Inner loop updated for {device_id}")


def handle_outer_pid_update(device_id, pg, dg, ig, sp, rot):
    update_pid_values(
        device_id=device_id,
        loop="outer",
        pg=pg,
        dg=dg,
        ig=ig,
        sp=sp,
        rot=rot
    )
    print(f"[PID] Outer loop updated for {device_id}")
