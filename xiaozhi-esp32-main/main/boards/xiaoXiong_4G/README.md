# xiaoXiong_4G

这个 board 按 `docs/custom-board.md` 的方式新建，当前功能与 `atoms3-echo-base` 保持一致。

编译方式可参考：

```bash
idf.py set-target esp32s3
idf.py menuconfig
```

在菜单中选择：

```text
Xiaozhi Assistant -> Board Type -> xiaoXiong_4G
```

也可以直接使用：

```bash
python scripts/release.py xiaoXiong_4G
```
