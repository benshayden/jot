print('before collect:')
print('used: ' + str(gc.mem_alloc()))
print('free: ' + str(gc.mem_free()))
gc.collect()
print('after collect:')
print('used: ' + str(gc.mem_alloc()))
print('free: ' + str(gc.mem_free()))
